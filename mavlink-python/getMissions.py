from pymavlink import mavutil

# Verbindung herstellen
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Warten auf Heartbeat
master.wait_heartbeat()
print(f"Heartbeat empfangen von System (sysid {master.target_system}, compid {master.target_component})")

# Missionsanzahl anfragen
master.mav.mission_request_list_send(master.target_system, master.target_component)

# Warten auf Mission Count
msg = master.recv_match(type='MISSION_COUNT', blocking=True)
count = msg.count
print(f"Anzahl Missionen: {count}")

# Jede Mission einzeln anfragen
missions = []
for i in range(count):
    master.mav.mission_request_int_send(master.target_system, master.target_component, i)
    msg = master.recv_match(type='MISSION_ITEM_INT', blocking=True)
    missions.append(msg)

# Alle Missionen ausgeben
for i, m in enumerate(missions):
    print(f"\nMission {i}:")
    print(f"  seq: {m.seq}")
    print(f"  frame: {m.frame}")
    print(f"  command: {m.command}")
    print(f"  params: {[m.param1, m.param2, m.param3, m.param4]}")
    print(f"  x (lat*1e7): {m.x}, y (lon*1e7): {m.y}, z (alt): {m.z}")
