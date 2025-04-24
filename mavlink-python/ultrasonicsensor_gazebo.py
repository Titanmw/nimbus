import gz.transport13 as gz_transport
from gz.msgs11 import laserscan_pb2
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("MAVLink Heartbeat empfangen.")

start_time = time.time()

def callback(msg):
    time_boot_ms = int((time.time() - start_time) * 1000)
    ranges = list(msg.ranges)

    if not ranges:
        print("Keine Range-Daten erhalten.")
        return

    distance_m = ranges[len(ranges)//2]

    if 0 < distance_m < float('inf'):
        distance_cm = int(distance_m * 100)
        print(f"\r📡 LaserScan → {distance_cm} cm @ {time_boot_ms} ms", end="", flush=True)
    else:
        print("\rUngültige Messung: kein gültiger Wert", end="", flush=True)
        distance_cm = 65535  # Ungültig / kein Echo

    master.mav.distance_sensor_send(
        time_boot_ms=time_boot_ms,
        min_distance=20,
        max_distance=400,
        current_distance=distance_cm,
        type=0,
        id=1,
        orientation=0,
        covariance=0
    )


node = gz_transport.Node()
topic = "/world/iris_runway/model/iris_with_gimbal/link/ultrasound_front_link/sensor/ultrasound_front/scan"

success = node.subscribe(
    msg_type=laserscan_pb2.LaserScan,
    topic=topic,
    callback=callback
)

if not success:
    print("Konnte nicht auf Topic abonnieren:", topic)
else:
    print(f"Lausche auf Topic: {topic}")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nAbbruch durch Benutzer")
