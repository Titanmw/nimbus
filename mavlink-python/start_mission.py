from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14552')
master.wait_heartbeat()
print("Verbindung steht!")

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

print("Copter sollte nun die Mission abfliegen...")
