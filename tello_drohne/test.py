import threading
from djitellopy import tello
import time


me = tello.Tello()
me.connect()

print(f"Battery: {me.get_battery()} %")
time.sleep(1)
me.takeoff()
me.move_left(30)
me.land()
