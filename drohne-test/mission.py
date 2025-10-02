from pymavlink import mavutil
import time

def connect_to_drone():
    """Verbindung zur Drohne herstellen."""
    try:
        connection = mavutil.mavlink_connection(
            'COM6', baud=57600  # Port und Baudrate anpassen
        )
        print("Verbindung hergestellt.")
        return connection
    except Exception as e:
        print(f"Fehler beim Verbinden: {e}")
        return None

def wait_for_heartbeat(connection):
    """Warten auf den Heartbeat der Drohne."""
    print("Warten auf Heartbeat...")
    connection.wait_heartbeat()
    print("Heartbeat empfangen. Verbindung zur Drohne aktiv!")

def clear_all_waypoints(connection):
    """Alle gespeicherten Wegpunkte löschen."""
    print("Lösche alle gespeicherten Missionen/Wegpunkte...")
    connection.mav.mission_clear_all_send(connection.target_system, connection.target_component)
    print("Alle Wegpunkte erfolgreich gelöscht.")

def upload_mission(connection, lat, lon, alt):
    """Neue Mission hinzufügen."""
    print("Erstelle neue Mission...")
    mission_items = [
        mavutil.mavlink.MAVLink_mission_item_message(
            connection.target_system,
            connection.target_component,
            0,  # Sequenznummer
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Relativer Höhenrahmen
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Wegpunkt-Befehl
            1,  # Aktueller Wegpunkt
            1,  # Automatisch fortfahren
            0, 0, 0, 0,  # Keine weiteren Parameter
            lat,  # Latitude in Grad
            lon,  # Longitude in Grad
            alt   # Höhe in Metern
        )
    ]

    print(f"Mission-Wegpunkt: Latitude={lat}, Longitude={lon}, Höhe={alt}m")
    connection.mav.mission_count_send(connection.target_system, connection.target_component, len(mission_items))

    for i, item in enumerate(mission_items):
        print(f"Sende Wegpunkt {i}: {item}")
        connection.mav.mission_item_send(
            connection.target_system, connection.target_component,
            i, item.frame, item.command,
            item.current, item.autocontinue,
            item.param1, item.param2, item.param3, item.param4,
            item.x, item.y, item.z
        )

    print("Warten auf Mission Bestätigung...")
    monitor_mission_flow(connection)

def monitor_mission_flow(connection):
    """Warten auf die Bestätigung der Mission."""
    while True:
        msg = connection.recv_match(type=None, blocking=True)
        if msg:
            if msg.get_type() == "MISSION_ACK":
                print("Mission erfolgreich bestätigt!")
                break

def check_gps_fix(connection):
    """Prüfen, ob die Drohne GPS-Daten liefert."""
    print("Überprüfe GPS-Daten...")
    while True:
        msg = connection.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        if msg:
            latitude = msg.lat / 1e7
            longitude = msg.lon / 1e7
            altitude = msg.alt / 1000
            if latitude != 0.0 and longitude != 0.0:
                print(f"GPS-Fix vorhanden: Latitude={latitude}, Longitude={longitude}, Höhe={altitude}m")
                return
            else:
                print("Kein gültiger GPS-Fix. Warte auf GPS...")

def read_all_missions(connection):
    """Alle gespeicherten Missionen aus der Drohne auslesen."""
    print("Fordere Missionsliste von der Drohne an...")
    connection.mav.mission_request_list_send(connection.target_system, connection.target_component)

    # Warten auf MISSION_COUNT
    msg = connection.recv_match(type="MISSION_COUNT", blocking=True)
    if not msg:
        print("Keine Missionen gefunden.")
        return

    mission_count = msg.count
    print(f"Anzahl der gespeicherten Wegpunkte: {mission_count}")

    # Wegpunkte abrufen
    for i in range(mission_count):
        print(f"Fordere Wegpunkt {i} an...")
        connection.mav.mission_request_int_send(connection.target_system, connection.target_component, i)
        msg = connection.recv_match(type="MISSION_ITEM_INT", blocking=True)
        if msg:
            latitude = msg.x / 1e7  # Konvertiere Latitude in Grad
            longitude = msg.y / 1e7  # Konvertiere Longitude in Grad
            altitude = msg.z  # Höhe in Metern
            print(f"Wegpunkt {i}: Latitude={latitude}, Longitude={longitude}, Höhe={altitude}m")
        else:
            print(f"Wegpunkt {i} konnte nicht abgerufen werden.")

def disconnect_drone(connection):
    """Verbindung zur Drohne trennen."""
    if connection:
        print("Verbindung zur Drohne wird getrennt...")
        connection.close()
        print("Verbindung erfolgreich getrennt.")
    else:
        print("Keine aktive Verbindung gefunden.")

def print_system_ids(connection):
    print("Überprüfe System- und Komponenten-IDs...")
    connection.mav.heartbeat_send(6, 8, 0, 0, 0)  # Beispiel für MAVLink Heartbeat
    heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True)
    if heartbeat:
        print(f"Target System: {heartbeat.get_srcSystem()}, Target Component: {heartbeat.get_srcComponent()}")


if __name__ == "__main__":
    # GPS-Koordinaten der U-Bahn-Station Alt Erlaa
    ALT_ERLAA_LAT = 48.146290
    ALT_ERLAA_LON = 16.287330
    ALTITUDE = 10  # Zielhöhe in Metern

    # Verbindung herstellen
    drone_connection = connect_to_drone()

    if drone_connection:
        try:
            wait_for_heartbeat(drone_connection)

            # GPS funktinoiert noch nicht
            # check_gps_fix(drone_connection)

            read_all_missions(drone_connection)

            clear_all_waypoints(drone_connection)

            # upload_mission(drone_connection, ALT_ERLAA_LAT, ALT_ERLAA_LON, ALTITUDE)

            read_all_missions(drone_connection)
        finally:
            disconnect_drone(drone_connection)
