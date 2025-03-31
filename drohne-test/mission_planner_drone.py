import asyncio
from mavsdk import System

async def connect_drone():
    """Verbindet sich mit der Drohne und wartet auf Bestätigung"""
    drone = System(mavsdk_server_address="localhost", port=50051)
    print("🔗 Verbinde mit der Drohne...")
    await drone.connect()

    # Warte, bis die Verbindung tatsächlich aufgebaut ist
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Verbindung zur Drohne hergestellt!")
            break
    return drone  # Gibt die verbundene Drohne zurück

async def fly_mission():
    """Drohne hebt ab und fliegt eine einfache Mission"""
    drone = await connect_drone()  # Warten auf erfolgreiche Verbindung

    print("⚡ Arming der Drohne...")
    await drone.action.arm()

    print("🚀 Takeoff auf 10m Höhe...")
    await drone.action.takeoff()

    await asyncio.sleep(5)

    print("🛬 Landeanflug...")
    await drone.action.land()

    print("✅ Drohne ist gelandet!")

asyncio.run(fly_mission())
