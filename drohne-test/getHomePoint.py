from pymavlink import mavutil

def get_waypoints():
    try:
        # Verbindung zum Fahrzeug herstellen (z.B. via UART oder UDP)
        master = mavutil.mavlink_connection('COM6', baud=57600)

        # Auf das erste Herzschlag-Signal warten, um die Verbindung zu bestätigen
        print("Warte auf Verbindung...")
        master.wait_heartbeat()
        print("Verbindung hergestellt!")

        # Anfrage für den Missions-Count senden
        print("Sende Anfrage für Missions-Count...")
        master.mav.mission_request_list_send(master.target_system, master.target_component)

        # Warten auf die Antwort mit der Anzahl der Wegpunkte
        mission_count_message = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=10)
        if not mission_count_message:
            print("Timeout beim Empfang der Missions-Count-Daten.")
            return

        mission_count = mission_count_message.count
        print(f"Anzahl der Wegpunkte: {mission_count}")

        # Wegpunkte abrufen
        waypoints = []
        for i in range(mission_count):
            print(f"Anfrage für Wegpunkt {i}...")
            master.mav.mission_request_int_send(master.target_system, master.target_component, i)
            message = master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=10)
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
        print(f"Ein Fehler ist aufgetreten: {e}")

if __name__ == "__main__":
    get_waypoints()
