import time
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import ImageTk, Image
import threading
import cv2
from djitellopy import tello
import mediapipe as mp
import numpy as np

button_width = 8
# button_height = 4

connected = False
send_control_flag = False
face_detection = False
face_center = False
qr_detection = False
qr_code_center = False

last_command_id = None
qr_controlled = False
main_bg_color = '#2E2E2E'

def create_darkmode_styles():
    style = ttk.Style()
    style.theme_use('clam')  # Verwende ein modernes Theme

    # Definiere die Hintergrundfarbe, die du für die "Transparenz" simulieren willst
    parent_bg_color = '#2E2E2E'  # Gleiche Hintergrundfarbe wie das übergeordnete Widget (z.B. Frame)

    # Farben und Schriftarten für Buttons und Labels im Darkmode
    style.configure('TButton', font=('Helvetica', 12, 'bold'), background='#333333', foreground='#00BFFF', padding=10)
    style.map('TButton', background=[('active', '#444444')], foreground=[('active', '#00BFFF')])

    # Labels und LabelFrame Schriftfarbe auf Weiß (#FFFFFF) ändern und Hintergrund an das übergeordnete Widget anpassen
    style.configure('TLabel', font=('Helvetica', 12), background=parent_bg_color, foreground='#FFFFFF', padding=5)
    style.configure('TFrame', background=parent_bg_color)

    # Der LabelFrame hat den gleichen Hintergrund wie das übergeordnete Widget für den transparenten Effekt
    style.configure('TLabelFrame', background=parent_bg_color, foreground='#FFFFFF', font=('Helvetica', 14, 'bold'))

    # Textfelder ebenfalls anpassen
    style.configure('TText', background='#1E1E1E', foreground='#00BFFF')


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

    offset_x_threshold = 200
    offset_y_threshold = 200

    if abs(offset[0]) > offset_x_threshold:
        if offset[0] > 0:
            print("Drohne nach rechts bewegen")
            me.send_rc_control(20, 0, 0, 0)
        else:
            print("Drohne nach links bewegen")
            me.send_rc_control(-20, 0, 0, 0)

    if abs(offset[1]) > offset_y_threshold:
        if offset[1] > 0:
            print("Drohne nach unten bewegen")
            me.send_rc_control(0, 0, -20 , 0)
        else:
            print("Drohne nach oben bewegen")
            me.send_rc_control(0, 0, 20, 0)

    size_threshold = 30
    if abs(size_error) > size_threshold:
        if size_error > 0:
            print("Drohne rückwärts bewegen")
            me.send_rc_control(0, -20, 0, 0)
        else:
            print("Drohne vorwärts bewegen")
            me.send_rc_control(0, 20, 0, 0)

def process_qr_command(decoded_text):
    global me, last_command_id

    # QR-Code wird in drei Teile gesplittet: COMMAND:<ACTION>:<ID>
    parts = decoded_text.split(":")

    # Prüfen, ob der QR-Code das richtige Format für einen Command hat
    if len(parts) == 3 and parts[0].upper() == "COMMAND":
        action = f"{parts[0].upper()}:{parts[1].upper()}"
        command_id = parts[2]

        # Überprüfen, ob der Befehl bereits ausgeführt wurde
        if command_id == last_command_id:
            print(f"Befehl {command_id} wurde bereits ausgeführt, wird ignoriert.")
            return
        else:
            print(f"Verarbeite neuen Befehl: {command_id}")
            last_command_id = command_id  # Speichern der aktuellen Command-ID

        # Drohnenbefehle basierend auf dem Command
        if action == "COMMAND:LAND":
            print("Befehl: Landen")
            threading.Thread(target=me.land, daemon=True).start()
        elif action == "COMMAND:FORWARD":
            print("Befehl: Vorwärts")
            threading.Thread(target=me.move_forward, daemon=True, args=[20]).start()
        elif action == "COMMAND:LEFT":
            print("Befehl: Links")
            threading.Thread(target=me.move_left, daemon=True, args=[20]).start()
        elif action == "COMMAND:RIGHT":
            print("Befehl: Rechts")
            threading.Thread(target=me.move_right, daemon=True, args=[20]).start()
        elif action == "COMMAND:BACKWARD":
            print("Befehl: Rückwärts")
            threading.Thread(target=me.move_back, daemon=True, args=[20]).start()
        elif action == "COMMAND:FLIP FORWARD":
            print("Befehl: Vorwärts")
            threading.Thread(target=me.flip_forward, daemon=True).start()
        elif action == "COMMAND:FLIP LEFT":
            print("Befehl: Links")
            threading.Thread(target=me.flip_left, daemon=True).start()
        elif action == "COMMAND:FLIP RIGHT":
            print("Befehl: Rechts")
            threading.Thread(target=me.flip_right, daemon=True).start()
        elif action == "COMMAND:FLIP BACKWARD":
            print("Befehl: Rückwärts")
            threading.Thread(target=me.flip_back, daemon=True).start()
        elif action == "COMMAND:UP":
            print("Befehl: Hoch")
            threading.Thread(target=me.move_up, daemon=True, args=[20]).start()
        elif action == "COMMAND:DOWN":
            print("Befehl: Runter")
            threading.Thread(target=me.move_down, daemon=True, args=[20]).start()
        elif action == "COMMAND:ROTATE LEFT":
            print("Befehl: Links Drehen")
            threading.Thread(target=me.rotate_counter_clockwise, daemon=True, args=[30]).start()
        elif action == "COMMAND:ROTATE RIGHT":
            print("Befehl: Rechts Drehen")
            threading.Thread(target=me.rotate_clockwise, daemon=True, args=[30]).start()
        else:
            print(f"Unbekannter Befehl: {action}")

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
                    text_qr_code.delete("1.0", tk.END)
                    text_qr_code.insert(tk.END, decoded_text)
                    text_qr_code.config(state="disabled")
                    text_qr_code.update()

                    # Process QR command if the format is COMMAND:<ACTION>
                    if qr_controlled and decoded_text.startswith("COMMAND:"):
                        process_qr_command(decoded_text)

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
    default_img = Image.open("./default_image.png")
    default_img = default_img.resize((960, 720))
    imgtk = ImageTk.PhotoImage(image=default_img)
    image_capture.config(image=imgtk)
    image_capture.image = imgtk
    drone_battery.configure(text='-')
    drone_status.update()

def button_face_detection_toggle():
    global face_detection
    if face_detection:
        face_detection = False
        drone_face_detect.configure(text='OFF')
    else:
        face_detection = True
        drone_face_detect.configure(text='ON')

    drone_face_detect.update()

def button_face_center_toggle():
    global face_center
    if face_center:
        face_center = False
        drone_face_center.configure(text='OFF')
    else:
        face_center = True
        drone_face_center.configure(text='ON')

    drone_face_center.update()

def button_qr_detect_toggle():
    global qr_detection
    if qr_detection:
        qr_detection = False
        drone_qr_detect.configure(text='OFF')
        text_qr_code.config(state="normal")
        text_qr_code.delete("1.0", tk.END)
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
    else:
        qr_code_center = True
        drone_qr_center.configure(text='ON')

    drone_qr_center.update()

def button_qr_controlled_toggle():
    global qr_controlled

    if qr_controlled:
        qr_controlled = False
        drone_qr_controlled.configure(text='OFF')
    else:
        qr_controlled = True
        drone_qr_controlled.configure(text='ON')

    drone_qr_controlled.update()

me = tello.Tello()

window = tk.Tk()
window.title("Tello Drone Controller")
frame = ttk.Frame(window)
frame.pack(fill="both", expand=True)

create_darkmode_styles()

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
mp_face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)


# Drone Frame
drone_frame = tk.LabelFrame(frame, text="Drone", background=main_bg_color, fg="white")
drone_frame.grid(row=0, column=0, sticky="news", padx=10, pady=10)

drone_status_label = ttk.Label(drone_frame, text="Status")
drone_status_label.grid(row=0, column=0)

drone_status = ttk.Label(drone_frame, text="offline")
drone_status.grid(row=0, column=1, padx=10, pady=10)

drone_connect_button = ttk.Button(drone_frame, text="Connect", command=button_connect_drone)
drone_connect_button.grid(row=0, column=2, padx=10, pady=10)

drone_battery_label = ttk.Label(drone_frame, text="Battery")
drone_battery_label.grid(row=1, column=0)

drone_battery = ttk.Label(drone_frame, text="-")
drone_battery.grid(row=1, column=1, padx=10, pady=10)

drone_height_label = ttk.Label(drone_frame, text="Height")
drone_height_label.grid(row=2, column=0)

drone_height = ttk.Label(drone_frame, text="-")
drone_height.grid(row=2, column=1, padx=10, pady=10)

drone_face_detect_label = ttk.Label(drone_frame, text="Face Detection")
drone_face_detect_label.grid(row=3, column=0)

drone_face_detect = ttk.Label(drone_frame, text="OFF")
drone_face_detect.grid(row=3, column=1, padx=10, pady=10)

drone_face_detect_button = ttk.Button(drone_frame, text="Toggle", command=button_face_detection_toggle)
drone_face_detect_button.grid(row=3, column=2, padx=10, pady=10)

drone_face_center_label = ttk.Label(drone_frame, text="Center Face")
drone_face_center_label.grid(row=4, column=0)

drone_face_center = ttk.Label(drone_frame, text="OFF")
drone_face_center.grid(row=4, column=1, padx=10, pady=10)

drone_face_button = ttk.Button(drone_frame, text="Toggle", command=button_face_center_toggle)
drone_face_button.grid(row=4, column=2, padx=10, pady=10)

drone_qr_detect_label = ttk.Label(drone_frame, text="QR-Code Detection")
drone_qr_detect_label.grid(row=5, column=0)

drone_qr_detect = ttk.Label(drone_frame, text="OFF")
drone_qr_detect.grid(row=5, column=1, padx=10, pady=10)

drone_qr_detect_button = ttk.Button(drone_frame, text="Toggle", command=button_qr_detect_toggle)
drone_qr_detect_button.grid(row=5, column=2, padx=10, pady=10)

drone_qr_center_label = ttk.Label(drone_frame, text="Center QR-Code")
drone_qr_center_label.grid(row=6, column=0)

drone_qr_center = ttk.Label(drone_frame, text="OFF")
drone_qr_center.grid(row=6, column=1, padx=10, pady=10)

drone_qr_center_button = ttk.Button(drone_frame, text="Toggle", command=button_qr_center_toggle)
drone_qr_center_button.grid(row=6, column=2, padx=10, pady=10)

drone_qr_controlled_label = ttk.Label(drone_frame, text="QR-Code conrolled")
drone_qr_controlled_label.grid(row=7, column=0)

drone_qr_controlled = ttk.Label(drone_frame, text="OFF")
drone_qr_controlled.grid(row=7, column=1, padx=10, pady=10)

drone_qr_controlled_button = ttk.Button(drone_frame, text="Toggle", command=button_qr_controlled_toggle)
drone_qr_controlled_button.grid(row=7, column=2, padx=10, pady=10)

drone_takeoff_button = ttk.Button(drone_frame, text="Take off", width=button_width,
                                  command=lambda: threading.Thread(target=me.takeoff, daemon=True).start())
drone_takeoff_button.grid(row=8, column=0, padx=10, pady=10)

drone_land_button = ttk.Button(drone_frame, text="Land", width=button_width,
                               command=lambda: threading.Thread(target=me.land, daemon=True).start())
drone_land_button.grid(row=8, column=1, padx=10, pady=10)

# Control Frame
def start_sending_rc_control(left_right, forward_backward, up_down, yaw):
    print("start rc")

    def send_rc_control_continuously():
        global send_control_flag

        while send_control_flag:  # Continue sending commands while the flag is True
            me.send_rc_control(left_right, forward_backward, up_down, yaw)
            time.sleep(0.1)
        
        me.send_rc_control(0, 0, 0, 0)
    global send_control_flag
    send_control_flag = True
    threading.Thread(target=send_rc_control_continuously, daemon=True).start()

# Define the function to stop sending RC control values when the button is released
def stop_sending_rc_control():
    global send_control_flag
    print("stop rc")

    send_control_flag = False

control_frame = tk.LabelFrame(drone_frame, text="Controls", background=main_bg_color, fg="white")
control_frame.grid(row=9, column=0, columnspan=2, padx=10, pady=10)

# Forward button
control_forward = ttk.Button(control_frame, text="Forward", width=button_width)
control_forward.grid(row=1, column=1, padx=10, pady=10)
control_forward.bind('<ButtonPress-1>', lambda event: start_sending_rc_control(0, 20, 0, 0))
control_forward.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Left button
control_left = ttk.Button(control_frame, text="Left", width=button_width)
control_left.grid(row=2, column=0, padx=10, pady=10)
control_left.bind('<ButtonPress-1>', lambda event: start_sending_rc_control(-20, 0, 0, 0))
control_left.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Right button
control_right = ttk.Button(control_frame, text="Right", width=button_width)
control_right.grid(row=2, column=2, padx=10, pady=10)
control_right.bind('<ButtonPress-1>', lambda event: start_sending_rc_control(20, 0, 0, 0))
control_right.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Back button
control_back = ttk.Button(control_frame, text="Back", width=button_width)
control_back.grid(row=3, column=1, padx=10, pady=10)
control_back.bind('<ButtonPress-1>', lambda event: start_sending_rc_control(0, -20, 0, 0))
control_back.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Up button
control_up = ttk.Button(control_frame, text="Up", width=button_width)
control_up.grid(row=1, column=3, padx=10, pady=10)
control_up.bind('<ButtonPress-1>', lambda event: start_sending_rc_control(0, 0, 20, 0))
control_up.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Down button
control_down = ttk.Button(control_frame, text="Down", width=button_width)
control_down.grid(row=3, column=3, padx=10, pady=10)
control_down.bind('<ButtonPress-1>', lambda event: start_sending_rc_control(0, 0, -20, 0))
control_down.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Rotate Left button
control_rotate_left = ttk.Button(control_frame, text="Rotate Left", width=button_width)
control_rotate_left.grid(row=1, column=0, padx=10, pady=10)
control_rotate_left.bind('<ButtonPress-1>', lambda event: start_sending_rc_control(0, 0, 0, -30))
control_rotate_left.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Rotate Right button
control_rotate_right = ttk.Button(control_frame, text="Rotate Right", width=button_width)
control_rotate_right.grid(row=1, column=2, padx=10, pady=10)
control_rotate_right.bind('<ButtonPress-1>', lambda event: start_sending_rc_control(0, 0, 0, 30))
control_rotate_right.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())


# Flip Frame
flip_frame = tk.LabelFrame(drone_frame, text="Flip", background=main_bg_color, fg="white")
flip_frame.grid(row=9, column=2, padx=10, pady=10)

flip_forward = ttk.Button(flip_frame, text="Forward", width=button_width,
                          command=lambda: threading.Thread(target=me.flip_forward, daemon=True).start())
flip_forward.grid(row=1, column=1, padx=10, pady=10)

flip_left = ttk.Button(flip_frame, text="Left", width=button_width,
                       command=lambda: threading.Thread(target=me.flip_left, daemon=True).start())
flip_left.grid(row=2, column=0, padx=10, pady=10)

flip_right = ttk.Button(flip_frame, text="Right", width=button_width,
                        command=lambda: threading.Thread(target=me.flip_right, daemon=True).start())
flip_right.grid(row=2, column=2, padx=10, pady=10)

flip_backward = ttk.Button(flip_frame, text="Backward", width=button_width,
                           command=lambda: threading.Thread(target=me.flip_back, daemon=True).start())
flip_backward.grid(row=3, column=1, padx=10, pady=10)


# QR Frame
qr_label = ttk.Label(drone_frame, text="QR Code Value")
qr_label.grid(row=10, column=0, padx=10, pady=10)

text_qr_code = tk.Text(drone_frame, height=5, width=30, background=main_bg_color, fg="white")
text_qr_code.grid(row=10, column=1, columnspan=2, sticky="ew", padx=10, pady=10)
text_qr_code.config(state="disabled")


# Bild Frame
image_frame = tk.LabelFrame(frame, text="Image", background=main_bg_color, fg="white")
image_frame.grid(row=0, column=1, sticky="news", padx=10, pady=10)

image_capture = ttk.Label(image_frame, text="Capture")
image_capture.grid(row=0, column=0, padx=10, pady=10, sticky="ew")


# Main GUI Loop
show_default_image()

window.configure(bg=main_bg_color)
window.mainloop()
