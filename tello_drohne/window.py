import tkinter
from tkinter import ttk, messagebox
from PIL import ImageTk, Image
import threading
import cv2
from djitellopy import tello
import mediapipe as mp
import numpy as np

connected = False
face_detection = False
face_center = False
qr_detection = False
qr_code_center = False

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

def calculate_offset_and_size(points, image):
    if points is None or len(points) < 4:
        return None, None

    center_x = np.mean([point[0] for point in points])
    center_y = np.mean([point[1] for point in points])

    image_center_x = image.shape[1] / 2
    image_center_y = image.shape[0] / 2

    offset_x = center_x - image_center_x
    offset_y = center_y - image_center_y

    size = 0
    num_points = len(points)

    for i in range(num_points):
        pt1 = points[i]
        pt2 = points[(i + 1) % num_points]
        distance = np.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
        size += distance

    size /= num_points

    return (offset_x, offset_y), size

def adjust_drone_position(offset, size, desired_size):
    global me

    if not me.is_flying:
        return

    size_error = size - desired_size

    offset_x_threshold = 100
    offset_y_threshold = 100

    if abs(offset[0]) > offset_x_threshold:
        if offset[0] > 0:
            print("Drohne nach rechts bewegen")
            me.send_rc_control(offset_x_threshold, 0, 0, 0)
        else:
            print("Drohne nach links bewegen")
            me.send_rc_control(-offset_x_threshold, 0, 0, 0)

    if abs(offset[1]) > offset_y_threshold:
        if offset[1] > 0:
            print("Drohne nach unten bewegen")
            me.send_rc_control(0, 0, -offset_y_threshold, 0)
        else:
            print("Drohne nach oben bewegen")
            me.send_rc_control(0, 0, offset_y_threshold, 0)

    size_threshold = 30
    if abs(size_error) > size_threshold:
        if size_error > 0:
            print("Drohne rückwärts bewegen")
            me.send_rc_control(0, -20, 0, 0)
        else:
            print("Drohne vorwärts bewegen")
            me.send_rc_control(0, 20, 0, 0)

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

        if qr_detection:
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

                    offset, size = calculate_offset_and_size(points, image)

                    text_offset = f"Offset (x, y): ({offset[0]:.2f}, {offset[1]:.2f})"
                    text_size = f"Size: {size:.2f} px"

                    cv2.putText(image, text_offset, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(image, text_size, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)

                    if qr_code_center and me.is_flying:
                        adjust_drone_position(offset, size, 250)

        if face_detection:
            results = mp_face_detection.process(image)
            if results.detections:
                for detection in results.detections:
                    mp_drawing.draw_detection(image, detection)
                    face_box = detection.location_data.relative_bounding_box
                    image_height, image_width, _ = image.shape
                    points = [
                        (int(face_box.xmin * image_width), int(face_box.ymin * image_height)),
                        (int((face_box.xmin + face_box.width) * image_width), int(face_box.ymin * image_height)),
                        (int((face_box.xmin + face_box.width) * image_width), int((face_box.ymin + face_box.height) * image_height)),
                        (int(face_box.xmin * image_width), int((face_box.ymin + face_box.height) * image_height)),
                    ]

                    offset, size = calculate_offset_and_size(points, image)

                    text_offset = f"Offset (x, y): ({offset[0]:.2f}, {offset[1]:.2f})"
                    text_size = f"Size: {size:.2f} px"

                    cv2.putText(image, text_offset, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(image, text_size, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)

                    if face_center and me.is_flying:
                        adjust_drone_position(offset, size, 300)

        im_pil = Image.fromarray(image)
        imgtk = ImageTk.PhotoImage(image=im_pil)

        image_capture.config(image=imgtk)
        image_capture.image = imgtk
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

def button_face_detection_toggle():
    global face_detection
    if face_detection:
        face_detection = False
        drone_face_detect.configure(text='OFF')
        drone_face_detect.update()
    else:
        face_detection = True
        drone_face_detect.configure(text='ON')
        drone_face_detect.update()

def button_face_center_toggle():
    global face_center
    if face_center:
        face_center = False
        drone_face_center.configure(text='OFF')
        drone_face_center.update()
    else:
        face_center = True
        drone_face_center.configure(text='ON')
        drone_face_center.update()

def button_qr_detect_toggle():
    global qr_detection
    if qr_detection:
        qr_detection = False
        drone_qr_detect.configure(text='OFF')
        drone_qr_detect.update()
        text_qr_code.config(state="normal")
        text_qr_code.delete("1.0", tkinter.END)
        text_qr_code.config(state="disabled")
        text_qr_code.update()
    else:
        qr_detection = True
        drone_qr_detect.configure(text='ON')
        drone_qr_detect.update()

def button_qr_center_toggle():
    global qr_code_center
    if qr_code_center:
        qr_code_center = False
        drone_qr_center.configure(text='OFF')
        drone_qr_center.update()
    else:
        qr_code_center = True
        drone_qr_center.configure(text='ON')
        drone_qr_center.update()

me = tello.Tello()

window = tkinter.Tk()
window.title("Tello Drone Controller")
frame = tkinter.Frame(window)
frame.pack()

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
mp_face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)

# Drone Frame
drone_frame = tkinter.LabelFrame(frame, text="Drone")
drone_frame.grid(row=0, column=0, sticky="news", padx=20, pady=20)

drone_status_label = tkinter.Label(drone_frame, text="Status")
drone_status_label.grid(row=0, column=0)

drone_status = tkinter.Label(drone_frame, text="offline")
drone_status.grid(row=0, column=1, padx=10, pady=10)

drone_connect_button = tkinter.Button(drone_frame, text="Connect", command=button_connect_drone)
drone_connect_button.grid(row=0, column=2, padx=10, pady=10)

drone_battery_label = tkinter.Label(drone_frame, text="Battery")
drone_battery_label.grid(row=1, column=0)

drone_battery = tkinter.Label(drone_frame, text="-")
drone_battery.grid(row=1, column=1, padx=10, pady=10)

drone_height_label = tkinter.Label(drone_frame, text="Height")
drone_height_label.grid(row=2, column=0)

drone_height = tkinter.Label(drone_frame, text="-")
drone_height.grid(row=2, column=1, padx=10, pady=10)

drone_face_detect_label = tkinter.Label(drone_frame, text="Face Detection")
drone_face_detect_label.grid(row=3, column=0)

drone_face_detect = tkinter.Label(drone_frame, text="OFF")
drone_face_detect.grid(row=3, column=1, padx=10, pady=10)

drone_face_detect_button = tkinter.Button(drone_frame, text="Toggle", command=button_face_detection_toggle)
drone_face_detect_button.grid(row=3, column=2, padx=10, pady=10)

drone_face_center_label = tkinter.Label(drone_frame, text="Center Face")
drone_face_center_label.grid(row=4, column=0)

drone_face_center = tkinter.Label(drone_frame, text="OFF")
drone_face_center.grid(row=4, column=1, padx=10, pady=10)

drone_face_button = tkinter.Button(drone_frame, text="Toggle", command=button_face_center_toggle)
drone_face_button.grid(row=4, column=2, padx=10, pady=10)

drone_qr_detect_label = tkinter.Label(drone_frame, text="QR-Code Detection")
drone_qr_detect_label.grid(row=5, column=0)

drone_qr_detect = tkinter.Label(drone_frame, text="OFF")
drone_qr_detect.grid(row=5, column=1, padx=10, pady=10)

drone_qr_detect_button = tkinter.Button(drone_frame, text="Toggle", command=button_qr_detect_toggle)
drone_qr_detect_button.grid(row=5, column=2, padx=10, pady=10)

drone_qr_center_label = tkinter.Label(drone_frame, text="Center QR-Code")
drone_qr_center_label.grid(row=6, column=0)

drone_qr_center = tkinter.Label(drone_frame, text="OFF")
drone_qr_center.grid(row=6, column=1, padx=10, pady=10)

drone_qr_button = tkinter.Button(drone_frame, text="Toggle", command=button_qr_center_toggle)
drone_qr_button.grid(row=6, column=2, padx=10, pady=10)

# Control Frame

button_width = 8
button_height = 4

control_frame = tkinter.LabelFrame(drone_frame, text="Controls")
control_frame.grid(row=8, column=0, padx=20, pady=20)

control_forward = tkinter.Button(control_frame, text="Forward", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.move_forward, daemon=True, args=[10]).start())
control_forward.grid(row=1, column=1, padx=10, pady=10)

control_left = tkinter.Button(control_frame, text="Left", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.move_left, daemon=True, args=[10]).start())
control_left.grid(row=2, column=0, padx=10, pady=10)

control_right = tkinter.Button(control_frame, text="Right", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.move_right, daemon=True, args=[10]).start())
control_right.grid(row=2, column=2, padx=10, pady=10)

control_back = tkinter.Button(control_frame, text="Back", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.move_back, daemon=True, args=[10]).start())
control_back.grid(row=3, column=1, padx=10, pady=10)

control_up = tkinter.Button(control_frame, text="Up", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.move_up, daemon=True, args=[10]).start())
control_up.grid(row=1, column=3, padx=10, pady=10)

control_down = tkinter.Button(control_frame, text="Down", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.move_down, daemon=True, args=[10]).start())
control_down.grid(row=3, column=3, padx=10, pady=10)

# Takeoff and Land buttons above the controls
drone_takeoff_button = tkinter.Button(drone_frame, text="Take off", width=button_width, height=button_height, command=button_takeoff_drone)
drone_takeoff_button.grid(row=7, column=0, padx=10, pady=10)

drone_land_button = tkinter.Button(drone_frame, text="Land", width=button_width, height=button_height, command=button_land_drone)
drone_land_button.grid(row=7, column=1, padx=10, pady=10)

# Flip Frame placed next to the control buttons (to the right)
flip_frame = tkinter.LabelFrame(drone_frame, text="Flip")
flip_frame.grid(row=8, column=1, padx=20, pady=20)

flip_forward = tkinter.Button(flip_frame, text="Forward", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.flip_forward, daemon=True).start())
flip_forward.grid(row=1, column=1, padx=10, pady=10)

flip_left = tkinter.Button(flip_frame, text="Left", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.flip_left, daemon=True).start())
flip_left.grid(row=2, column=0, padx=10, pady=10)

flip_right = tkinter.Button(flip_frame, text="Right", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.flip_right, daemon=True).start())
flip_right.grid(row=2, column=2, padx=10, pady=10)

flip_backward = tkinter.Button(flip_frame, text="Backward", width=button_width, height=button_height, command=lambda: threading.Thread(target=me.flip_back, daemon=True).start())
flip_backward.grid(row=3, column=1, padx=10, pady=10)

# QR Frame
qr_code_frame = tkinter.LabelFrame(drone_frame, text="QR Code")
qr_code_frame.grid(row=9, column=0, columnspan=2, sticky="news", padx=20, pady=20)

text_qr_code = tkinter.Text(qr_code_frame, height=5, width=30)
text_qr_code.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
text_qr_code.config(state="disabled")

# Bild Frame
image_frame = tkinter.LabelFrame(frame, text="Image")
image_frame.grid(row=0, column=1, sticky="news", padx=20, pady=20)

image_capture = tkinter.Label(image_frame, text="Capture")
image_capture.grid(row=0, column=0, padx=10, pady=10, sticky="ew")


# Main GUI Loop
show_default_image()

window.mainloop()
