from pymavlink import mavutil

def get_waypoints(master):
    """Holt alle vorhandenen Wegpunkte aus der Drohne."""
    print("Sende Anfrage für Missions-Count...")
    master.mav.mission_request_list_send(master.target_system, master.target_component)

    mission_count_message = master.recv_match(type='MISSION_COUNT', blocking=True, timeout=10)
    if not mission_count_message:
        print("Timeout beim Empfang der Missions-Count-Daten.")
        return []

    mission_count = mission_count_message.count
    print(f"Anzahl der bestehenden Wegpunkte: {mission_count}")

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
                'altitude': message.z,
            })
            print(f"Wegpunkt {i} erhalten: {waypoints[-1]}")
        else:
            print(f"Timeout beim Empfang von Wegpunkt {i}.")

    return waypoints

def upload_waypoints(master, waypoints):
    """Lädt eine neue Missionsliste mit aktualisierten Wegpunkten hoch."""
    print("Lösche bestehende Mission...")
    master.mav.mission_clear_all_send(master.target_system, master.target_component)

    print(f"Senden der neuen Mission mit {len(waypoints)} Wegpunkten...")
    master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))

    for i, wp in enumerate(waypoints):
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            i,  # Reihenfolge anpassen
            wp['frame'],
            wp['command'],
            0,  # Keine Bestätigung erforderlich
            0,  # AutoContinue deaktiviert
            0, 0, 0, 0,  # Param 1-4 (optional)
            int(wp['latitude'] * 1e7),
            int(wp['longitude'] * 1e7),
            wp['altitude']
        )
        print(f"Wegpunkt {i} gesendet: {wp}")

    print("Alle neuen Wegpunkte erfolgreich hochgeladen.")

def add_single_waypoint(latitude, longitude, altitude):
    try:
        # Verbindung zur Drohne herstellen
        master = mavutil.mavlink_connection('COM13', baud=115200)
        print("Warte auf Verbindung...")
        master.wait_heartbeat()
        print("Verbindung hergestellt!")

        # Bestehende Wegpunkte abrufen
        waypoints = get_waypoints(master)

        # Neuen Wegpunkt hinzufügen
        new_waypoint = {
            'seq': len(waypoints),  # Neuer Wegpunkt bekommt die nächste Sequenznummer
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude
        }
        waypoints.append(new_waypoint)
        print(f"Neuer Wegpunkt hinzugefügt: {new_waypoint}")

        # Neue Missionsliste hochladen
        upload_waypoints(master, waypoints)

    except Exception as e:
        print(f"Ein Fehler ist aufgetreten: {e}")

if __name__ == "__main__":
    # Beispiel: Einen neuen Wegpunkt in Wien mit 50m Höhe hinzufügen
    add_single_waypoint(48.2202472, 16.4440374, 50)
