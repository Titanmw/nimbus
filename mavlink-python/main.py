from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
master.wait_heartbeat()
print("Heartbeat empfangen.")

master.set_mode_apm("GUIDED")
time.sleep(2)

master.mav.param_set_send(
    master.target_system,
    master.target_component,
    b'ARMING_CHECK',  # Parametername als bytes
    0,  # Neuer Wert (float/int)
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)
time.sleep(1)  # etwas warten, bis der FC den Param gesetzt hat

# 4) Arme den Copter
master.arducopter_arm()
print("Copter arm-Befehl geschickt.")

master.set_mode_apm("AUTO")
time.sleep(2)

master.motors_armed_wait()
print("Armed!")

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_MISSION_START, # 300
    0, # confirmation
    0, # first_item = 0
    0, # last_item = 0 (falls 0 => bis letztes WP)
    0, 0, 0, 0, 0  # unused
)

boot_time = time.time()

while True:
    time_boot_ms = int((time.time() - boot_time) * 1000)
    master.mav.distance_sensor_send(
        time_boot_ms,
        20,     # min cm
        500,    # max cm
        150,    # current cm
        mavutil.mavlink.MAV_DISTANCE_SENSOR_ULTRASOUND,      # type: 1 = Ultrasound
        0,      # id
        0,      # orientation (forward)
        0,      # covariance (unknown)
        20,     # horizontal FOV (deg)
        20,     # vertical FOV (deg)
        [0, 0, 0, 1],  # quaternion (unit rotation)
        255     # signal quality (max)
    )
    time.sleep(0.1)  # 10 Hz
