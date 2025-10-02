from pymavlink import mavutil

def get_gps_signal():
    try:
        # Verbindung zum Fahrzeug herstellen (z.B. via UART oder UDP)
        master = mavutil.mavlink_connection('COM13', baud=115200)

        # Auf das erste Herzschlag-Signal warten, um die Verbindung zu bestätigen
        print("Warte auf Verbindung...")
        master.wait_heartbeat()
        print("Verbindung hergestellt!")

        # GPS-Daten abrufen
        print("Warte auf GPS-Daten...")
        gps_message = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=10)

        if gps_message:
            lat = gps_message.lat / 1e7  # Umrechnung in Dezimalgrad
            lon = gps_message.lon / 1e7  # Umrechnung in Dezimalgrad
            alt = gps_message.alt / 1000.0  # Umrechnung in Meter
            sat_count = gps_message.satellites_visible  # Anzahl sichtbarer Satelliten
            fix_type = gps_message.fix_type  # GPS-Fix-Typ (0 = kein Fix, 3 = 3D Fix)

            print(f"GPS-Daten erhalten:")
            print(f"Breitengrad: {lat}")
            print(f"Längengrad: {lon}")
            print(f"Höhe: {alt} m")
            print(f"Sichtbare Satelliten: {sat_count}")
            print(f"Fix-Typ: {fix_type} (0=Kein Fix, 2=2D Fix, 3=3D Fix)")

        else:
            print("Kein GPS-Signal erhalten.")

    except Exception as e:
        print(f"Ein Fehler ist aufgetreten: {e}")

if __name__ == "__main__":
    get_gps_signal()
