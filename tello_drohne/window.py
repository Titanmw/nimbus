# default_image.png from "https://www.flaticon.com/free-icons/no"
import tkinter
from tkinter import ttk, messagebox
from PIL import ImageTk, Image
import threading
import cv2
from djitellopy import tello
import mediapipe as mp
import numpy as np

connected = False
ai = False

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

def calculate_qr_code_offset_and_size(image, points):
    if points is None or len(points) < 4:
        return None, None  # Keine gültigen QR-Code-Punkte

    qr_center_x = np.mean([point[0] for point in points])
    qr_center_y = np.mean([point[1] for point in points])

    image_center_x = image.shape[1] / 2
    image_center_y = image.shape[0] / 2

    offset_x = qr_center_x - image_center_x
    offset_y = qr_center_y - image_center_y

    qr_size = 0
    num_points = len(points)

    for i in range(num_points):
        pt1 = points[i]
        pt2 = points[(i + 1) % num_points]
        distance = np.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
        qr_size += distance

    qr_size /= num_points  # Durchschnittliche Kantenlänge des QR-Codes

    return (offset_x, offset_y), qr_size

def adjust_drone_position(offset, qr_size):
    global me

    if not me.is_flying:
        return
    """
    Funktion zur Anpassung der Drohnenposition, um den QR-Code in die Mitte zu bewegen und
    die Größe auf etwa 300 Pixel einzustellen.
    """

    desired_qr_size = 300  # Zielgröße des QR-Codes in Pixeln
    size_error = qr_size - desired_qr_size

    # Steuerkommandos, um den QR-Code zur Mitte zu bewegen
    # Offset-Werte sind in Pixeln, wir müssen sie in Drohnenbewegungen übersetzen
    offset_x_threshold = 20  # Schwellenwert für die X-Verschiebung in Pixeln
    offset_y_threshold = 20  # Schwellenwert für die Y-Verschiebung in Pixeln

    # Bewege Drohne basierend auf X-Offset
    if abs(offset[0]) > offset_x_threshold:
        if offset[0] > 0:
            print("Drohne nach rechts bewegen")
            me.send_rc_control(offset_x_threshold, 0, 0, 0)  # Bewege nach rechts
        else:
            print("Drohne nach links bewegen")
            me.send_rc_control(-offset_x_threshold, 0, 0, 0)  # Bewege nach links

    # Bewege Drohne basierend auf Y-Offset
    if abs(offset[1]) > offset_y_threshold:
        if offset[1] > 0:
            print("Drohne nach unten bewegen")
            me.send_rc_control(0, 0, -offset_y_threshold, 0)  # Bewege nach unten
        else:
            print("Drohne nach oben bewegen")
            me.send_rc_control(0, 0, offset_y_threshold, 0)  # Bewege nach oben

    # Größe anpassen (Vorwärts/Zurück basierend auf der QR-Code-Größe)
    size_threshold = 30  # Toleranz für die Größenabweichung in Pixeln
    if abs(size_error) > size_threshold:
        if size_error > 0:
            print("Drohne rückwärts bewegen")
            me.send_rc_control(0, -20, 0, 0)  # Rückwärts bewegen
        else:
            print("Drohne vorwärts bewegen")
            me.send_rc_control(0, 20, 0, 0)  # Vorwärts bewegen

def update_drone_image():
    global connected
    qr_detector = cv2.QRCodeDetector()

    while connected:
        battery_text = "-"
        try:
            battery_text = me.get_battery()
        except Exception as e:
            disconnect_to_drone()
            break

        drone_battery.configure(text=f'{battery_text}%')
        drone_frame.update()
        drone_height.configure(text=f'{me.get_height()} cm')
        drone_height.update()

        image = me.get_frame_read().frame
        if image is None:
            print("Kein Bild vom Video-Stream empfangen.")
            continue

        image.flags.writeable = True
        # image = cv2.flip(image, 1)

        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # image = cv2.putText(image, f'Battery: {me.battery_text}%', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        decoded_text, points, _ = qr_detector.detectAndDecode(image)

        if points is not None:
            points = points[0]

            if decoded_text:
                for i in range(len(points)):
                    pt1 = tuple(map(int, points[i]))
                    pt2 = tuple(map(int, points[(i + 1) % len(points)]))
                    cv2.line(image, pt1, pt2, color=(0, 255, 0), thickness=2)

                text_qr_code.config(state="normal")
                text_qr_code.delete("1.0", tkinter.END)
                text_qr_code.insert(tkinter.END, decoded_text)
                text_qr_code.config(state="disabled")
                text_qr_code.update()

                offset, size = calculate_qr_code_offset_and_size(image, points)

                text_offset = f"Offset (x, y): ({offset[0]:.2f}, {offset[1]:.2f})"
                text_size = f"Size: {size:.2f} px"

                cv2.putText(image, text_offset, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(image, text_size, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)

                print(f"Abweichung des QR-Codes: {offset}")
                print(f"Größe des QR-Codes: {size}")

                if me.is_flying:
                    adjust_drone_position(offset, size)
        else:
            text_qr_code.config(state="normal")
            text_qr_code.delete("1.0", tkinter.END)
            text_qr_code.config(state="disabled")
            text_qr_code.update()

        if ai:
            results = face_detection.process(image)
            if results.detections:
                for detection in results.detections:
                    mp_drawing.draw_detection(image, detection)

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

def button_takeoff_drone():
    global me

    threading.Thread(target=me.takeoff, daemon=True).start()

def button_land_drone():
    global me

    threading.Thread(target=me.land, daemon=True).start()

def button_ai_toggle():
    global ai, drone_ai

    if ai:
        ai = False
        drone_ai.configure(text='OFF')
        drone_ai.update()
    else:
        ai = True
        drone_ai.configure(text='ON')
        drone_ai.update()


me = tello.Tello()

window = tkinter.Tk()
window.title("Tello Drone Controller")
frame = tkinter.Frame(window)
frame.pack()

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

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

drone_height_label = tkinter.Label(drone_frame, text="Height")
drone_height_label.grid(row=2, column=0)

drone_height = tkinter.Label(drone_frame, text="-")
drone_height.grid(row=2, column=1, padx=10, pady=10)

drone_ai_label = tkinter.Label(drone_frame, text="AI")
drone_ai_label.grid(row=3, column=0)

drone_ai = tkinter.Label(drone_frame, text="OFF")
drone_ai.grid(row=3, column=1, padx=10, pady=10)

drone_connect_button = tkinter.Button(drone_frame, text="Connect", command=button_connect_drone)
drone_connect_button.grid(row=4, column=0, padx=10, pady=10)

drone_ai_button = tkinter.Button(drone_frame, text="Turn AI ON", command=button_ai_toggle)
drone_ai_button.grid(row=4, column=1, padx=10, pady=10)

drone_takeoff_button = tkinter.Button(drone_frame, text="Take off", command=button_takeoff_drone)
drone_takeoff_button.grid(row=5, column=0, padx=10, pady=10)

drone_land_button = tkinter.Button(drone_frame, text="Land", command=button_land_drone)
drone_land_button.grid(row=5, column=1, padx=10, pady=10)

# Bild Frame
image_frame = tkinter.LabelFrame(frame, text="Image")
image_frame.grid(row=0, column=1, sticky="news", padx=20, pady=20)

image_capture = tkinter.Label(image_frame, text="Capture")
image_capture.grid(row=0, column=0, padx=10, pady=10, sticky="ew")

show_default_image()

qr_code_frame = tkinter.LabelFrame(frame, text="QR Code")
qr_code_frame.grid(row=1, column=0, sticky="news", padx=20, pady=20)

text_qr_code = tkinter.Text(qr_code_frame, height=5, width=30)
text_qr_code.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
text_qr_code.config(state="disabled")

# Main GUI Loop
window.mainloop()
