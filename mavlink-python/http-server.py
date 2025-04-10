from flask import Flask, request, jsonify
from pymavlink import mavutil
import threading
import queue
import time
import uuid

app = Flask(__name__)

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
    global missions

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
    global status, missions

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

    while True:
        now = time.time()

        # 1x pro Sekunde Heartbeat senden
        if now - last_heartbeat > 1:
            send_gcs_heartbeat()
            last_heartbeat = now

        # Nachrichten empfangen
        msg = master.recv_match(blocking=False)
        if msg:
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
                print(f"📩 COMMAND_ACK empfangen für Command {msg.command} mit Ergebnis {msg.result}")

                # Optionale Antwort in die response_queue legen
                response_queue.put({
                    "command": msg.command,
                    "result": msg.result,
                    "result_text": mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
                })

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

                # ✅ Jetzt sauber in die Queue mit request_id
                response_queue.put({
                    "request_id": req_id,
                    "missions": new_missions
                })

            elif cmd["action"] == "add_waypoint":
                missions.append(cmd["waypoint"])

                update_missions(master)

            elif cmd["action"] == "delete_waypoint":
                seq = cmd["seq"]

                # Suche und lösche Waypoint
                updated = [wp for wp in missions if wp["seq"] != seq]

                # Neu nummerieren
                for i, wp in enumerate(updated):
                    wp["seq"] = i

                missions[:] = updated
                update_missions(master)

            elif cmd["action"] == "clear_missions":
                master.mav.mission_clear_all_send(master.target_system, master.target_component)
                
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

            elif cmd["action"] == "start_mission":
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_MISSION_START,
                    0,
                    0, 0, 0, 0, 0, 0, 0
                )


        except queue.Empty:
            pass

        time.sleep(0.01)

# --- Flask API-Endpunkte ---

@app.route("/getWayPoints", methods=["GET"])
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

@app.route("/getStatus", methods=["GET"])
def get_status():
    return jsonify(status)

@app.route("/addWayPoint", methods=["POST"])
def add_waypoint():
    data = request.get_json()

    # Validierung
    if not all(k in data for k in ("latitude", "longitude", "altitude", "command", "params")):
        return jsonify({"error": "Missing parameters"}), 400
    if not isinstance(data["params"], list) or len(data["params"]) != 4:
        return jsonify({"error": "params must be a list of 4 values"}), 400

    command_queue.put({
        "action": "add_waypoint",
        "waypoint": {
            "latitude": data["latitude"],
            "longitude": data["longitude"],
            "altitude": data["altitude"],
            "command": data["command"],
            "params": data["params"]
        }
    })
    return jsonify({"status": "Waypoint added to mission queue"})

@app.route("/deleteWayPoint", methods=["POST", "DELETE"])
def delete_waypoint():
    data = request.get_json()
    seq_to_delete = data.get("seq")

    if seq_to_delete is None:
        return jsonify({"error": "Missing 'seq' parameter"}), 400

    command_queue.put({
        "action": "delete_waypoint",
        "seq": seq_to_delete
    })

    return jsonify({"status": f"Waypoint {seq_to_delete} marked for deletion"})

@app.route("/deleteAllWaypoints", methods=["DELETE"])
def delete_all_waypoints():
    command_queue.put({"action": "clear_missions"})
    return jsonify({"status": "All waypoints deleted"})

@app.route("/setMode", methods=["POST"])
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
                return jsonify({
                    "status": "ack",
                    "mode": mode,
                    "result": response["result"],
                    "result_text": response["result_text"]
                })
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

@app.route("/startMission", methods=["POST"])
def start_mission():
    command_queue.put({
        "action": "start_mission"
    })
    return jsonify({"status": "Mission start command sent"})

# --- Start Threads ---
threading.Thread(target=mavlink_worker, daemon=True).start()

if __name__ == "__main__":
    command_queue.put({"action": "get_missions"})
    app.run(host="0.0.0.0", port=5000)
