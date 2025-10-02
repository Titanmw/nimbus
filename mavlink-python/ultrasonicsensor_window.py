import tkinter as tk
from pymavlink import mavutil
import time
import threading

# MAVLink-Verbindung
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("Heartbeat empfangen.")

start_time = time.time()

# Funktion, die die MAVLink-Daten sendet
def send_distance():
    while True:
        time_boot_ms = int((time.time() - start_time) * 1000)
        distance_cm = slider.get()
        master.mav.distance_sensor_send(
            time_boot_ms,
            20,     # min_distance
            400,    # max_distance
            distance_cm,
            0,      # type: ultrasound
            1,      # sensor id
            0,      # orientation: forward
            0       # covariance
        )
        print(f"Sende: {distance_cm} cm @ {time_boot_ms} ms")
        time.sleep(0.1)

# GUI
root = tk.Tk()
root.title("Ultraschall-Distanzsimulator")

slider = tk.Scale(root, from_=20, to=400, orient=tk.HORIZONTAL, length=400, label="Distanz (cm)")
slider.set(150)
slider.pack(padx=20, pady=20)

# Sende-Thread starten
thread = threading.Thread(target=send_distance, daemon=True)
thread.start()

# GUI starten
root.mainloop()
