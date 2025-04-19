from flask import Flask, request, jsonify, Response
from pymavlink import mavutil
import threading
import queue
import time
import uuid
import cv2
import requests
import numpy as np

windows_ip = "172.22.32.1"

app = Flask(__name__)
cap = cv2.VideoCapture(0)

# Globale Variablen für Status & Missionen
status = {
    "voltage": 0.0,
    "current": 0.0,
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "gps_fix": 0,
    "satellites": 0,
    "mode": "UNKNOWN",
    "base_mode": 0,
    "custom_mode": 0,
    "system_status": "UNKNOWN"
}
missions = []

active_command = {}

# Kommando-Queue vom Webserver an den MAVLink-Thread
command_queue = queue.Queue()

# Antwort-Queue (z. B. für Waypoint-Daten)
response_queue = queue.Queue()

def wait_for_ack(request_id, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        try:
            response = response_queue.get(timeout=timeout)
            if response.get("request_id") == request_id:
                return response
        except queue.Empty:
            break
    return None

def update_missions(master: mavutil.mavlink_connection):
    global status, missions, active_command

    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    time.sleep(0.2)  # kleine Pause zur Sicherheit

    count = len(missions)
    master.mav.mission_count_send(master.target_system, master.target_component, count)
    time.sleep(0.1)

    for i, wp in enumerate(missions):
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            i,  # seq
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            wp["command"],
            0, 1,
            wp["params"][0],
            wp["params"][1],
            wp["params"][2],
            wp["params"][3],
            int(wp["latitude"] * 1e7),
            int(wp["longitude"] * 1e7),
            float(wp["altitude"]),
            0
        )
        time.sleep(0.05)  # kleine Pause, um Überlastung zu vermeiden

    # Optional: Auf ACK warten
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack:
        print(f"✅ Mission Upload bestätigt: {ack.type}")
    else:
        print("⚠️ Keine Bestätigung vom Fahrzeug erhalten")

def mavlink_worker():
    global status, missions, active_command

    print("🔌 Starte MAVLink-Verbindung...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
    master.wait_heartbeat()
    print(f"✅ Heartbeat empfangen von System {master.target_system}")

    # Heartbeat senden
    def send_gcs_heartbeat():
        master.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_GCS,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE
        )

    last_heartbeat = 0
    last_receive = time.time()

    while True:
        now = time.time()

        # 1x pro Sekunde Heartbeat senden
        if now - last_heartbeat > 1:
            send_gcs_heartbeat()
            last_heartbeat = now

        # Nachrichten empfangen
        msg = master.recv_match(blocking=False)
        if msg:
            last_receive = now  # Empfangszeit aktualisieren
            
            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT":
                status["base_mode"] = msg.base_mode
                status["custom_mode"] = msg.custom_mode
                status["system_status"] = msg.system_status
                mode_name = mavutil.mode_string_v10(msg)
                status["mode"] = mode_name
            elif msg_type == "GPS_RAW_INT":
                status["latitude"] = msg.lat / 1e7
                status["longitude"] = msg.lon / 1e7
                status["altitude"] = msg.alt / 1000
                status["gps_fix"] = msg.fix_type
                status["satellites"] = msg.satellites_visible
            elif msg_type == "BATTERY_STATUS":
                if msg.voltages[0] != 65535:
                    status["voltage"] = msg.voltages[0] / 1000
                if msg.current_battery != -1:
                    status["current"] = msg.current_battery / 100.0
            elif msg_type == "COMMAND_ACK":
                print(f"COMMAND_ACK empfangen für Command {msg.command} mit Ergebnis {msg.result}")
                
                if active_command and msg.command == active_command.get("command"):
                    response_queue.put({
                        "request_id": active_command["request_id"],
                        "command": msg.command,
                        "result": msg.result,
                        "result_text": mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
                    })
                    active_command = {}  # Reset nach ACK
            elif msg_type == "MISSION_ACK":
                print(f"MISSION_ACK empfangen mit Ergebnis {msg.type}")

                if active_command and active_command.get("command") == "CLEAR_MISSIONS":
                    response_queue.put({
                        "request_id": active_command["request_id"],
                        "result": msg.type,
                        "result_text": mavutil.mavlink.enums['MAV_MISSION_RESULT'][msg.type].name
                    })
                    active_command = {}


        # ⏰ Reconnect nach 10 Sekunden ohne Empfang
        if now - last_receive > 10:
            print("⚠️ Keine MAVLink-Nachrichten seit 10s – reconnect...")
            try:
                master.close()
            except Exception:
                pass
            time.sleep(1)
            master = connect()
            last_receive = time.time()

        # Webserver-Kommandos abarbeiten
        try:
            cmd = command_queue.get_nowait()
            if cmd["action"] == "get_missions":
                req_id = cmd.get("request_id")
                
                master.mav.mission_request_list_send(master.target_system, master.target_component)
                count_msg = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=3)
                new_missions = []

                if count_msg:
                    for i in range(count_msg.count):
                        master.mav.mission_request_int_send(master.target_system, master.target_component, i)
                        item = master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=3)
                        if item:
                            new_missions.append({
                                "seq": item.seq,
                                "frame": item.frame,
                                "command": item.command,
                                "params": [item.param1, item.param2, item.param3, item.param4],
                                "latitude": item.x / 1e7,
                                "longitude": item.y / 1e7,
                                "altitude": item.z
                            })

                missions[:] = new_missions

                response_queue.put({
                    "request_id": req_id,
                    "missions": new_missions
                })

            elif cmd["action"] == "clear_missions":
                missions.clear()
                active_command = {
                    "request_id": cmd.get("request_id"),
                    "command": "CLEAR_MISSIONS"
                }

                master.mav.mission_clear_all_send(master.target_system, master.target_component)

            elif cmd["action"] == "set_missions":
                missions[:] = cmd["waypoints"]
                # Neu nummerieren
                for i, wp in enumerate(missions):
                    wp["seq"] = i
                update_missions(master)
                
            elif cmd["action"] == "set_mode":
                mode_string = cmd["mode"]
                request_id = cmd.get("request_id")
                mode_id = master.mode_mapping().get(mode_string)

                if mode_id is not None:
                    # master.mav.command_long_send(
                    #     master.target_system,
                    #     master.target_component,
                    #     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    #     0,
                    #     mode_id, 0, 0, 0, 0, 0, 0
                    # )
                    master.set_mode(mode_id)

                    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                        response_queue.put({
                            "request_id": request_id,
                            "result": ack.result,
                            "result_text": mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
                        })
                    else:
                        response_queue.put({
                            "request_id": request_id,
                            "error": "No ACK received from vehicle"
                        })

            elif cmd["action"] == "arm":
                req_id = cmd.get("request_id")

                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1, 0, 0, 0, 0, 0, 0
                )

                ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    response_queue.put({
                        "request_id": req_id,
                        "result": ack.result,
                        "result_text": mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
                    })

            elif cmd["action"] == "disarm":
                req_id = cmd.get("request_id")
                force = cmd.get("force", False)
                force_code = 21196 if force else 0

                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    0,  # Disarm
                    force_code,
                    0, 0, 0, 0, 0
                )

                ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    response_queue.put({
                        "request_id": req_id,
                        "result": ack.result,
                        "result_text": mavutil.mavlink.enums['MAV_RESULT'][ack.result].name
                    })
                else:
                    response_queue.put({
                        "request_id": req_id,
                        "error": "No ACK received"
                    })


            elif cmd["action"] == "start_mission":
                active_command = {
                    "request_id": cmd.get("request_id"),
                    "command": mavutil.mavlink.MAV_CMD_MISSION_START
                }

                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_MISSION_START,
                    0,
                    0, 0, 0, 0, 0, 0, 0
                )

            elif cmd["action"] == "goto_position":
                master.mav.set_position_target_global_int_send(
                    0,  # time_boot_ms (0 = ignorieren)
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # relativer Rahmen
                    0b0000111111111000,  # type_mask: ignoriert vx, vy, vz, yaw_rate, accel
                    int(cmd["lat"] * 1e7),
                    int(cmd["lon"] * 1e7),
                    float(cmd["alt"]),
                    0, 0, 0,  # vx, vy, vz
                    0, 0, 0,  # afx, afy, afz
                    0, 0      # yaw, yaw_rate
                )

            elif cmd["action"] == "rotate":
                active_command = {
                    "request_id": cmd["request_id"],
                    "command": mavutil.mavlink.MAV_CMD_CONDITION_YAW
                }

                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                    0,
                    float(cmd["yaw"]),
                    float(cmd["speed"]),
                    int(cmd["direction"]),
                    1 if cmd.get("relative", True) else 0,  # relativ oder absolut
                    0, 0, 0
                )

            elif cmd["action"] == "guided_takeoff":
                active_command = {
                    "request_id": cmd.get("request_id"),
                    "command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                }

                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0,
                    0, 0, 0, 0, 0, 0,
                    float(cmd["altitude"])
                )

            elif cmd["action"] == "guided_land":
                active_command = {
                    "request_id": cmd.get("request_id"),
                    "command": mavutil.mavlink.MAV_CMD_NAV_LAND
                }

                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0,
                    0, 0, 0, 0, 0, 0, 0
                )



        except queue.Empty:
            pass

        time.sleep(0.01)

# --- Flask API-Endpunkte ---

@app.route("/get_missions", methods=["GET"])
def get_waypoints():
    req_id = str(uuid.uuid4())
    command_queue.put({"action": "get_missions", "request_id": req_id})
    try:
        while True:
            response = response_queue.get(timeout=5)
            if response.get("request_id") == req_id:
                return jsonify(response["missions"])
    except queue.Empty:
        return jsonify({"error": "Timeout bei Missionsabfrage"}), 504

@app.route("/get_status", methods=["GET"])
def get_status():
    return jsonify(status)

@app.route("/set_missions", methods=["POST"])
def add_waypoints():
    data = request.get_json()

    if not isinstance(data, list):
        return jsonify({"error": "Input must be a list of waypoint objects"}), 400

    for wp in data:
        if not all(k in wp for k in ("latitude", "longitude", "altitude", "command", "params")):
            return jsonify({"error": "Each waypoint must contain latitude, longitude, altitude, command, params"}), 400
        if not isinstance(wp["params"], list) or len(wp["params"]) != 4:
            return jsonify({"error": "params must be a list of 4 values"}), 400

    command_queue.put({
        "action": "set_missions",
        "waypoints": data
    })
    return jsonify({"status": f"{len(data)} waypoints added to mission queue"})

@app.route("/clear_missions", methods=["DELETE"])
def delete_all_waypoints():
    request_id = str(uuid.uuid4())

    command_queue.put({
        "action": "clear_missions",
        "request_id": request_id
    })

    ack = wait_for_ack(request_id, timeout=5)
    if ack:
        return jsonify({
            "ack": True,
            "result": ack["result"],
            "result_text": ack["result_text"]
        })
    else:
        return jsonify({"error": "No MISSION_ACK received"}), 504

@app.route("/set_mode", methods=["POST"])
def set_mode():
    data = request.get_json()
    mode = data.get("mode", "AUTO").upper()
    request_id = str(uuid.uuid4())

    command_queue.put({
        "action": "set_mode",
        "mode": mode,
        "request_id": request_id
    })

    try:
        while True:
            response = response_queue.get(timeout=5)
            if response.get("request_id") == request_id:
                if "result" in response:
                    return jsonify({
                        "status": "ack",
                        "mode": mode,
                        "result": response["result"],
                        "result_text": response["result_text"]
                    })
                else:
                    return jsonify({
                        "status": "nack",
                        "mode": mode,
                        "error": response.get("error", "Unbekannter Fehler")
                    }), 500

    except queue.Empty:
        return jsonify({"error": "No ACK received"}), 504

@app.route("/arm", methods=["POST"])
def arm_vehicle():
    request_id = str(uuid.uuid4())

    command_queue.put({
        "action": "arm",
        "request_id": request_id
    })

    ack = wait_for_ack(request_id, timeout=5)
    if ack:
        return jsonify({
            "ack": True,
            "result": ack["result"],
            "result_text": ack["result_text"]
        })
    else:
        return jsonify({"error": "No ACK received"}), 504

@app.route("/disarm", methods=["POST"])
def disarm_vehicle():
    data = request.get_json(silent=True) or {}
    force = data.get("force", False)
    request_id = str(uuid.uuid4())

    command_queue.put({
        "action": "disarm",
        "force": force,
        "request_id": request_id
    })

    ack = wait_for_ack(request_id, timeout=5)
    if ack:
        return jsonify({
            "ack": True,
            "result": ack["result"],
            "result_text": ack["result_text"]
        })
    else:
        return jsonify({"error": "No ACK received"}), 504

@app.route("/start_mission", methods=["POST"])
def start_mission():
    request_id = str(uuid.uuid4())

    command_queue.put({
        "action": "start_mission",
        "request_id": request_id
    })

    ack = wait_for_ack(request_id, timeout=5)
    if ack:
        return jsonify({
            "ack": True,
            "result": ack["result"],
            "result_text": ack["result_text"]
        })
    else:
        return jsonify({"error": "No ACK received"}), 504

@app.route("/camera", methods=["GET"])
def get_camera_image():
    try:
        stream_url = f"http://{windows_ip}:8080/camera"  # Windows-Stream
        response = requests.get(stream_url, stream=True, timeout=5)

        # Suche nach einem vollständigen JPEG-Bild im Stream
        buffer = b""
        for chunk in response.iter_content(chunk_size=1024):
            buffer += chunk
            start = buffer.find(b'\xff\xd8')  # JPEG Start
            end = buffer.find(b'\xff\xd9')    # JPEG Ende

            if start != -1 and end != -1 and end > start:
                jpg = buffer[start:end+2]
                img_array = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

                if frame is None:
                    return jsonify({"error": "Fehler beim Decodieren des Bildes"}), 500

                ret, jpeg = cv2.imencode(".jpg", frame)
                if not ret:
                    return jsonify({"error": "Fehler beim JPEG-Encoding"}), 500

                return Response(jpeg.tobytes(), mimetype='image/jpeg')

        return jsonify({"error": "Kein vollständiges JPEG im Stream gefunden"}), 500

    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route("/goto", methods=["POST"])
def goto():
    data = request.get_json()

    lat = data.get("latitude")
    lon = data.get("longitude")
    alt = data.get("altitude")

    if None in [lat, lon, alt]:
        return jsonify({"error": "Missing latitude, longitude or altitude"}), 400

    command_queue.put({
        "action": "goto_position",
        "lat": lat,
        "lon": lon,
        "alt": alt
    })

    return jsonify({"status": "goto_position enqueued"})

@app.route("/rotate", methods=["POST"])
def rotate():
    data = request.get_json()
    yaw = data.get("yaw")  # in Grad
    speed = data.get("speed", 10)
    direction = data.get("direction", 1)
    relative = data.get("relative", True)  # 🆕 Standardmäßig relativ

    if yaw is None:
        return jsonify({"error": "Missing yaw angle"}), 400

    request_id = str(uuid.uuid4())
    command_queue.put({
        "action": "rotate",
        "yaw": yaw,
        "speed": speed,
        "direction": direction,
        "relative": relative,
        "request_id": request_id
    })

    ack = wait_for_ack(request_id, timeout=5)
    if ack:
        return jsonify({
            "ack": True,
            "result": ack["result"],
            "result_text": ack["result_text"]
        })
    else:
        return jsonify({"error": "No ACK received"}), 504

@app.route("/takeoff", methods=["POST"])
def takeoff():
    data = request.get_json()
    altitude = data.get("altitude", 10.0)
    request_id = str(uuid.uuid4())

    command_queue.put({
        "action": "guided_takeoff",
        "altitude": altitude,
        "request_id": request_id
    })

    ack = wait_for_ack(request_id, timeout=5)
    if ack:
        return jsonify({
            "ack": True,
            "result": ack["result"],
            "result_text": ack["result_text"]
        })
    else:
        return jsonify({"error": "No ACK received"}), 504

@app.route("/land", methods=["POST"])
def land():
    request_id = str(uuid.uuid4())

    command_queue.put({
        "action": "guided_land",
        "request_id": request_id
    })

    ack = wait_for_ack(request_id, timeout=5)
    if ack:
        return jsonify({
            "ack": True,
            "result": ack["result"],
            "result_text": ack["result_text"]
        })
    else:
        return jsonify({"error": "No ACK received"}), 504



# --- Start Threads ---
threading.Thread(target=mavlink_worker, daemon=True).start()

if __name__ == "__main__":
    command_queue.put({"action": "get_missions"})
    app.run(host="0.0.0.0", port=5000)
