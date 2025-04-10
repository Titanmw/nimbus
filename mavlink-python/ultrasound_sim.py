import tkinter as tk
from pymavlink import mavutil
import time
import threading

# MAVLink-Verbindung aufbauen
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("✅ Heartbeat empfangen")

# Sensor-Konstanten
min_distance = 20     # cm
max_distance = 500    # cm
covariance = 0        # Ignoriert
sensor_type = 0       # Ignoriert
sensor_id = 0         # Ignoriert

# Initialisierungswerte
distance_cm = 100  # Initialer Abstand

def send_obstacle_distance():
    """Threaded-Senderoutine für OBSTACLE_DISTANCE"""
    while True:
        time_boot_ms = int(time.time() * 1000) % 2**32

        distances = [65535] * 72
        distances[0] = distance_cm  # direkt nach vorne

        master.mav.obstacle_distance_send(
            time_boot_ms,
            mavutil.mavlink.MAV_DISTANCE_SENSOR_ULTRASOUND,
            distances,
            20,
            500,
            25,
            5,  # 5° per step
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            0,
            0
        )

        time.sleep(0.2)

def on_slider_change(val):
    """Callback bei Slider-Änderung"""
    global distance_cm
    distance_cm = int(val)
    label_val.config(text=f"{distance_cm} cm")

# UI mit Tkinter
root = tk.Tk()
root.title("Ultraschall-Distanzsteuerung")

frame = tk.Frame(root, padx=20, pady=20)
frame.pack()

label = tk.Label(frame, text="Abstand zum Hindernis (cm):", font=("Arial", 14))
label.pack()

slider = tk.Scale(frame, from_=20, to=500, orient="horizontal", length=300, command=on_slider_change)
slider.set(distance_cm)
slider.pack()

label_val = tk.Label(frame, text=f"{distance_cm} cm", font=("Arial", 12))
label_val.pack()

# Starte den MAVLink-Sender in einem separaten Thread
threading.Thread(target=send_obstacle_distance, daemon=True).start()

root.mainloop()
