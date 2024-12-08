from flask import Flask, jsonify
from pymavlink import mavutil
import threading

app = Flask(__name__)

# Globale Variablen
drone_connection = None
battery_status = {}
missions = []

def connect_to_drone():
    """Verbindung zur Drohne herstellen."""
    global drone_connection
    try:
        drone_connection = mavutil.mavlink_connection('COM14', baud=115200)
        print("Verbindung hergestellt.")
        drone_connection.wait_heartbeat()
        print("Heartbeat empfangen. Verbindung zur Drohne aktiv!")
    except Exception as e:
        print(f"Fehler beim Verbinden: {e}")

def get_battery_status():
    """Batteriestand abfragen."""
    global battery_status
    if drone_connection:
        msg = drone_connection.recv_match(type="BATTERY_STATUS", blocking=True, timeout=2)
        if msg:
            battery_status = {
                "voltage": msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else None,  # Spannung in Volt
                "current": msg.current_battery / 100.0 if msg.current_battery != -1 else None,  # Strom in Ampere
                "remaining": msg.battery_remaining  # Batterie in Prozent
            }
        else:
            battery_status = {"error": "Keine Batterieinformationen verfügbar"}

def get_missions():
    """Missionen abrufen."""
    global missions
    missions = []
    if drone_connection:
        drone_connection.mav.mission_request_list_send(drone_connection.target_system, drone_connection.target_component)
        msg = drone_connection.recv_match(type="MISSION_COUNT", blocking=True, timeout=2)
        if msg:
            mission_count = msg.count
            for i in range(mission_count):
                drone_connection.mav.mission_request_int_send(drone_connection.target_system, drone_connection.target_component, i)
                wp_msg = drone_connection.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=2)
                if wp_msg:
                    missions.append({
                        "seq": wp_msg.seq,
                        "latitude": wp_msg.x / 1e7,
                        "longitude": wp_msg.y / 1e7,
                        "altitude": wp_msg.z
                    })
        else:
            missions = [{"error": "Keine Missionen verfügbar"}]

@app.route('/battery', methods=['GET'])
def battery_endpoint():
    """API-Endpunkt für den Batteriestand."""
    get_battery_status()
    return jsonify(battery_status)

@app.route('/missions', methods=['GET'])
def missions_endpoint():
    """API-Endpunkt für die Missionen."""
    get_missions()
    return jsonify(missions)

def start_web_server():
    """Webserver starten."""
    app.run(host='0.0.0.0', port=5000)

if __name__ == "__main__":
    # Drohnenverbindung in einem separaten Thread starten
    threading.Thread(target=connect_to_drone, daemon=True).start()

    # Webserver starten
    start_web_server()
