from flask import Flask, jsonify
from pymavlink import mavutil
import threading

app = Flask(__name__)

# Globale Variablen
drone_connection = None
battery_status = {}
waypoints = []

def connect_to_drone():
    """Verbindung zur Drohne herstellen."""
    global drone_connection
    try:
        # Verbindung über TCP mit Baudrate 115200 herstellen
        drone_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760', baud=115200)
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

def get_waypoints():
    """Wegpunkte abrufen."""
    global waypoints
    waypoints = []
    if drone_connection:
        try:
            print("Sende Anfrage für Missions-Count...")
            drone_connection.mav.mission_request_list_send(drone_connection.target_system, drone_connection.target_component)

            mission_count_message = drone_connection.recv_match(type='MISSION_COUNT', blocking=True, timeout=10)
            if not mission_count_message:
                print("Timeout beim Empfang der Missions-Count-Daten.")
                return

            mission_count = mission_count_message.count
            print(f"Anzahl der Wegpunkte: {mission_count}")

            for i in range(mission_count):
                print(f"Anfrage für Wegpunkt {i}...")
                drone_connection.mav.mission_request_int_send(drone_connection.target_system, drone_connection.target_component, i)
                message = drone_connection.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=10)
                if message:
                    waypoints.append({
                        'seq': message.seq,
                        'frame': message.frame,
                        'command': message.command,
                        'latitude': message.x / 1e7,
                        'longitude': message.y / 1e7,
                        'altitude': message.z
                    })
                    print(f"Wegpunkt {i}: {waypoints[-1]}")
                else:
                    print(f"Timeout beim Empfang von Wegpunkt {i}.")

            print("Alle Wegpunkte abgerufen:")
            for wp in waypoints:
                print(wp)
        except Exception as e:
            print(f"Fehler beim Abrufen der Wegpunkte: {e}")

@app.route('/battery', methods=['GET'])
def battery_endpoint():
    """API-Endpunkt für den Batteriestand."""
    get_battery_status()
    return jsonify(battery_status)

@app.route('/waypoints', methods=['GET'])
def waypoints_endpoint():
    """API-Endpunkt für die Wegpunkte."""
    get_waypoints()
    return jsonify(waypoints)

def start_web_server():
    """Webserver starten."""
    app.run(host='0.0.0.0', port=5000)

if __name__ == "__main__":
    # Drohnenverbindung in einem separaten Thread starten
    threading.Thread(target=connect_to_drone, daemon=True).start()

    # Webserver starten
    start_web_server()
