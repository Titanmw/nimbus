# default_image.png from "https://www.flaticon.com/free-icons/no"
import tkinter
from tkinter import ttk, messagebox
from PIL import ImageTk, Image
import threading
import cv2
from djitellopy import tello

connected = False

def button_connect_drone():
    global connected

    if connected:
        threading.Thread(target=disconnect_to_drone, daemon=True).start()
    else:
        threading.Thread(target=connect_to_drone, daemon=True).start()

def connect_to_drone():
    global connected
    global me

    print('Connecting to drone...')
    drone_status.configure(text='connecting')
    drone_status.update()

    try:
        me.connect(wait_for_state=True)
        me.streamon()

        connected = True
        drone_connect_button.config(text='Disconnect')
        drone_connect_button.update()
        battery_level = me.get_battery()
        print(f"Verbindung hergestellt! Batteriestand: {battery_level}%")
        drone_status.configure(text='connected')
        drone_status.update()

        threading.Thread(target=update_drone_image, daemon=True).start()

    except Exception as e:
        print(f"Fehler bei der Verbindung: {e}")
        drone_status.configure(text='offline')
        connected = False
        drone_connect_button.config(text='Connect')
        drone_connect_button.update()
        show_default_image()

def disconnect_to_drone():
    global connected
    global me

    if connected:
        connected = False
        drone_connect_button.config(text='Connect')
        drone_connect_button.update()

        try:
            me.streamoff()
        except Exception as e:
            print(f"Fehler beim Stoppen des Streams: {e}")

        try:
            me.end()
        except Exception as e:
            print(f"Fehler beim Trennen der Verbindung: {e}")

        me = None
        me = tello.Tello()
        show_default_image()
        drone_status.configure(text='offline')
        drone_status.update()

def update_drone_image():
    global connected

    while connected:
        battery_text = "-"
        try:
            battery_text = me.get_battery()
        except Exception as e:
            disconnect_to_drone()
            break

        drone_battery.configure(text=f'{battery_text}%')
        drone_frame.update()

        image = me.get_frame_read().frame
        if image is None:
            print("Kein Bild vom Video-Stream empfangen.")
            continue

        image.flags.writeable = True
        image = cv2.flip(image, 1)

        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # image = cv2.putText(image, f'Battery: {me.battery_text}%', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        im_pil = Image.fromarray(image)
        imgtk = ImageTk.PhotoImage(image=im_pil)

        # Bild im UI setzen
        image_capture.config(image=imgtk)
        image_capture.image = imgtk

        # Kurz warten, um das UI nicht zu blockieren
        image_capture.update()
        window.after(20)

def show_default_image():
    default_img = Image.open("default_image.png")
    default_img = default_img.resize((960, 720))
    imgtk = ImageTk.PhotoImage(image=default_img)
    image_capture.config(image=imgtk)
    image_capture.image = imgtk
    drone_battery.configure(text='-')
    drone_status.update()

me = tello.Tello()

window = tkinter.Tk()
window.title("Tello Drone Controller")
frame = tkinter.Frame(window)
frame.pack()

# Drone Frame
drone_frame = tkinter.LabelFrame(frame, text="Drone")
drone_frame.grid(row=0, column=0, sticky="news", padx=20, pady=20)

drone_status_label = tkinter.Label(drone_frame, text="Status")
drone_status_label.grid(row=0, column=0)

drone_status = tkinter.Label(drone_frame, text="offline")
drone_status.grid(row=0, column=1, padx=10, pady=10)

drone_battery_label = tkinter.Label(drone_frame, text="Battery")
drone_battery_label.grid(row=1, column=0)

drone_battery = tkinter.Label(drone_frame, text="-")
drone_battery.grid(row=1, column=1, padx=10, pady=10)

drone_connect_button = tkinter.Button(drone_frame, text="Connect", command=button_connect_drone)
drone_connect_button.grid(row=2, column=0, padx=10, pady=10)

# Bild Frame
image_frame = tkinter.LabelFrame(frame, text="Image")
image_frame.grid(row=0, column=1, sticky="news", padx=20, pady=20)

image_capture = tkinter.Label(image_frame, text="Capture")
image_capture.grid(row=0, column=0, padx=10, pady=10)

show_default_image()

# Main GUI Loop
window.mainloop()
