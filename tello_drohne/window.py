import tkinter as tk
from tkinter import ttk
from PIL import ImageTk, Image
import threading
import cv2
from djitellopy import tello
import mediapipe as mp
import numpy as np

button_width = 8
# button_height = 4
style = None

connected = False
face_detection = False
face_center = False
qr_detection = False
qr_code_center = False

is_proceeding_task = False

#Bojan Lösung
user_left_right = 0
user_forward_backward = 0
user_up_down = 0
user_yaw = 0

ai_left_right = 0
ai_forward_backward = 0
ai_up_down = 0
ai_yaw = 0

last_command_id = None
qr_controlled = False

main_bg_color = '#F0F0F0'  # Light background color

def create_lightmode_styles(frame):
    global style

    style = ttk.Style(frame)
    style.theme_use('clam')  # Verwende ein modernes Theme

    # Definiere die Hintergrundfarbe für das Light Theme
    parent_bg_color = '#F0F0F0'  # Helle Hintergrundfarbe

    # Farben und Schriftarten für Buttons (bleiben dunkel)
    style.configure('TButton', font=('Helvetica', 12, 'bold'), background='#333333', foreground='#00BFFF', padding=10)
    style.map('TButton', background=[('active', '#666666')], foreground=[('active', '#00BFFF')])

    # Labels und LabelFrame Schriftfarbe auf Dunkel (#000000) ändern und Hintergrund hell
    style.configure('TLabel', font=('Helvetica', 12), background=parent_bg_color, foreground='#000000', padding=5)
    style.configure('TFrame', background=parent_bg_color)

    # Der LabelFrame hat den gleichen Hintergrund wie das übergeordnete Widget für den hellen Effekt
    style.configure('TLabelFrame', background=parent_bg_color, foreground='#000000', font=('Helvetica', 14, 'bold'))

    # Textfelder ebenfalls anpassen
    style.configure('TText', background='#FFFFFF', foreground='#000000')

    style.configure("ON.TButton", background="green", foreground="white")
    style.map("ON.TButton", background=[("active", "darkgreen")])

    style.configure("OFF.TButton", background="red", foreground="white")
    style.map("OFF.TButton", background=[("active", "darkred")])

def show_error(message):
    error_log.config(state="normal")  # Aktiviere das Textfeld, um Text hinzuzufügen
    error_log.insert(tk.END, message + "\n")  # Füge die Fehlermeldung hinzu
    error_log.config(state="disabled")  # Deaktiviere das Textfeld, um Bearbeitungen zu verhindern
    error_log.see(tk.END)  # Scrolle nach unten, um die neueste Nachricht anzuzeigen

def check_drone_status():
    global connected

    if not connected:
        return  # Stoppe die Statusüberprüfung, wenn die Drohne nicht verbunden ist

    try:
        # Überprüfen des Batteriestands
        battery = me.get_battery()
        if battery < 20:
            show_error("Warnung: Der Batteriestand ist niedrig!")

        # Überprüfen der Flughöhe
        height = me.get_height()
        if height < 50:
            show_error("Warnung: Die Flughöhe ist sehr niedrig!")

    except Exception as e:
        show_error(f"Fehler beim Abrufen des Drohnenstatus: {e}")
        return

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
    drone_connect_button.config(text='Connecting')
    drone_connect_button.update()

    try:
        me.connect(wait_for_state=True)
        me.streamon()

        connected = True
        drone_connect_button.config(text='Disconnect')
        drone_connect_button.update()
        battery_level = me.get_battery()
        print(f"Verbindung hergestellt! Batteriestand: {battery_level}%")

        threading.Thread(target=update_drone_image, daemon=True).start()

    except Exception as e:
        print(f"Fehler bei der Verbindung: {e}")
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

def send_RC():
    global is_proceeding_task, me, user_left_right, user_forward_backward, user_up_down, user_yaw, ai_left_right, ai_forward_backward, ai_up_down, ai_yaw

    if not me.is_flying:
        return

    left_right = ai_left_right
    forward_backward = ai_forward_backward
    up_down = ai_up_down
    yaw = ai_yaw

    if is_proceeding_task:
        left_right = 0
        forward_backward = 0
        up_down = 0
        yaw = 0

    user_interaction = user_left_right != 0 or user_forward_backward != 0 or user_up_down != 0 or user_yaw != 0

    if user_interaction:
        left_right = user_left_right
        forward_backward = user_forward_backward
        up_down = user_up_down
        yaw = user_yaw
    
    ai_left_right = 0
    ai_forward_backward = 0
    ai_up_down = 0
    ai_yaw = 0

    if not is_proceeding_task or user_interaction:
        me.send_rc_control(left_right, forward_backward, up_down, yaw)

def start_sending_rc_control_user(left_right, forward_backward, up_down, yaw):
    global user_left_right, user_forward_backward, user_up_down, user_yaw
    
    user_left_right = left_right
    user_forward_backward = forward_backward
    user_up_down = up_down
    user_yaw = yaw
    
def start_sending_rc_control_ai(left_right, forward_backward, up_down, yaw):
    global ai_left_right, ai_forward_backward, ai_up_down, ai_yaw
    
    ai_left_right = left_right
    ai_forward_backward = forward_backward
    ai_up_down = up_down
    ai_yaw = yaw

def stop_sending_rc_control():
    global user_left_right, user_forward_backward, user_up_down, user_yaw

    user_left_right = 0
    user_forward_backward = 0
    user_up_down = 0
    user_yaw = 0

def adjust_drone_position(offset, size, desired_size):
    global me, send_control_flag

    if not me.is_flying:
        return

    size_error = size - desired_size

    offset_x_threshold = 40
    offset_y_threshold = 40
    size_threshold = 50

    left_right = 0
    forward_backward = 0
    up_down = 0

    if abs(offset[0]) > offset_x_threshold:
        if offset[0] > 0:
            print("Drohne nach rechts bewegen")
            left_right = 10
            # start_sending_rc_control(20, 0, 0, 0)
        else:
            print("Drohne nach links bewegen")
            left_right = -10
            # start_sending_rc_control(-20, 0, 0, 0)

    if abs(offset[1]) > offset_y_threshold:
        if offset[1] > 0:
            print("Drohne nach unten bewegen")
            up_down = -10
            # start_sending_rc_control(0, 0, -20 , 0)
        else:
            print("Drohne nach oben bewegen")
            up_down = 10
            # start_sending_rc_control(0, 0, 20, 0)

    if abs(size_error) > size_threshold:
        if size_error > 0:
            print("Drohne rückwärts bewegen")
            forward_backward = -10
            # start_sending_rc_control(0, -20, 0, 0)
        else:
            print("Drohne vorwärts bewegen")
            forward_backward = 10
            # start_sending_rc_control(0, 20, 0, 0)
    
    start_sending_rc_control_ai(left_right, forward_backward, up_down, 0)

def process_qr_command(decoded_text):
    global me, last_command_id, is_proceeding_task

    # QR-Code wird in mehrere Zeilen gesplittet
    lines = decoded_text.strip().split("\n")

    # Extrahiere die Programm-ID aus der ersten Zeile
    if len(lines) < 2:
        print("Ungültiges Format: Es muss eine Programm-ID und mindestens ein Befehl vorhanden sein.")
        return

    program_id = lines[0]
    print(f"Programm-ID: {program_id}")

    # Überprüfen, ob das Programm bereits ausgeführt wurde
    if program_id == last_command_id:
        print(f"Programm {program_id} wurde bereits ausgeführt, wird ignoriert.")
        return
    else:
        print(f"Verarbeite neues Programm: {program_id}")
        last_command_id = program_id  # Speichern der aktuellen Programm-ID

    is_proceeding_task = True

    # Verarbeite jeden Befehl in den folgenden Zeilen
    for line in lines[1:]:
        # Einzelnen Befehl verarbeiten
        parts = line.split(":")
        if len(parts) >= 1:
            action = parts[0].upper()
            args = parts[1:]  # Zusätzliche Argumente ab der 2. Stelle

            # Drohnenbefehle basierend auf dem Command
            try:
                if action == "LAND":
                    print("Befehl: Landen")
                    me.land()
                elif action == "FORWARD":
                    print(f"Befehl: Vorwärts, Argumente: {args}")
                    me.move_forward(int(args[0]))
                elif action == "LEFT":
                    print(f"Befehl: Links, Argumente: {args}")
                    me.move_left(int(args[0]))
                elif action == "RIGHT":
                    print(f"Befehl: Rechts, Argumente: {args}")
                    me.move_right(int(args[0]))
                elif action == "BACKWARD":
                    print(f"Befehl: Rückwärts, Argumente: {args}")
                    me.move_back(int(args[0]))
                elif action == "FLIP FORWARD":
                    print("Befehl: Flip Vorwärts")
                    me.flip_forward(int(args[0]))
                elif action == "FLIP LEFT":
                    print("Befehl: Flip Links")
                    me.flip_left()
                elif action == "FLIP RIGHT":
                    print("Befehl: Flip Rechts")
                    me.flip_right()
                elif action == "FLIP BACKWARD":
                    print("Befehl: Flip Rückwärts")
                    me.flip_back()
                elif action == "UP":
                    print(f"Befehl: Hoch, Argumente: {args}")
                    me.move_up(int(args[0]))
                elif action == "DOWN":
                    print(f"Befehl: Runter, Argumente: {args}")
                    me.move_down(int(args[0]))
                elif action == "ROTATE LEFT":
                    print(f"Befehl: Links Drehen, Argumente: {args}")
                    me.rotate_counter_clockwise(int(args[0]))
                elif action == "ROTATE RIGHT":
                    print(f"Befehl: Rechts Drehen, Argumente: {args}")
                    me.rotate_clockwise(int(args[0]))
                else:
                    print(f"Unbekannter Befehl: {action}")
            except (IndexError, ValueError) as e:
                print(f"Fehler beim Verarbeiten des Befehls: {line}. Fehler: {e}")
        else:
            print(f"Ungültiger Befehl: {line}")

    is_proceeding_task = False

def update_drone_image():
    global me, connected, is_proceeding_task
    qr_detector = cv2.QRCodeDetector()

    while connected:
        check_drone_status()
        
        battery_text = "Battery: - "
        try:
            battery_text = f"Battery: {me.get_battery()} %"
        except Exception as e:
            disconnect_to_drone()
            break

        image = me.get_frame_read().frame
        if image is None:
            print("Kein Bild vom Video-Stream empfangen.")
            continue

        image.flags.writeable = True
        height, width, _ = image.shape
        cv2.putText(image, f"Resolution: {width}x{height}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(image, battery_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)

        if qr_detection and image is not None and image.size != 0:
            try:
                decoded_text, points, _ = qr_detector.detectAndDecode(image)
                
                if points is not None:
                    point = points[0]

                    if decoded_text:
                        for i in range(len(point)):
                            pt1 = tuple(map(int, point[i]))
                            pt2 = tuple(map(int, point[(i + 1) % len(point)]))
                            cv2.line(image, pt1, pt2, color=(0, 255, 0), thickness=2)

                        text_qr_code.config(state="normal")
                        text_qr_code.delete("1.0", tk.END)
                        text_qr_code.insert(tk.END, decoded_text)
                        text_qr_code.config(state="disabled")
                        text_qr_code.update()

                        # Process QR command if the format is <ACTION>
                        if qr_controlled:
                            is_proceeding_task = True
                            threading.Thread(target=process_qr_command, daemon=True, args=[decoded_text]).start()

                        offset, size = calculate_offset_and_size(point, image)

                        text_offset = f"Offset (x, y): ({offset[0]:.2f}, {offset[1]:.2f})"
                        text_size = f"Size: {size:.2f} px"

                        cv2.putText(image, text_offset, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 1, cv2.LINE_AA)
                        cv2.putText(image, text_size, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 1, cv2.LINE_AA)

                        if qr_code_center and me.is_flying:
                            adjust_drone_position(offset, size, 70)
            except:
                print("QR Error")

        if face_detection and image is not None and image.size != 0:
            try:
                results = mp_face_detection.process(image)
                if results.detections:
                    for detection in results.detections:
                        mp_drawing.draw_detection(image, detection)
                        face_box = detection.location_data.relative_bounding_box
                        image_height, image_width, _ = image.shape
                        point = [
                            (int(face_box.xmin * image_width), int(face_box.ymin * image_height)),
                            (int((face_box.xmin + face_box.width) * image_width), int(face_box.ymin * image_height)),
                            (int((face_box.xmin + face_box.width) * image_width), int((face_box.ymin + face_box.height) * image_height)),
                            (int(face_box.xmin * image_width), int((face_box.ymin + face_box.height) * image_height)),
                        ]

                        offset, size = calculate_offset_and_size(point, image)

                        text_offset = f"Offset (x, y): ({offset[0]:.2f}, {offset[1]:.2f})"
                        text_size = f"Size: {size:.2f} px"

                        cv2.putText(image, text_offset, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)
                        cv2.putText(image, text_size, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2, cv2.LINE_AA)

                        if face_center and me.is_flying:
                            adjust_drone_position(offset, size, 70)
            except e:
                print("Face Detection Error")

        send_RC()

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

def button_face_detection_toggle():
    global face_detection, drone_face_detect_button

    if face_detection:
        face_detection = False
        drone_face_detect_button.configure(text='OFF', style="OFF.TButton")
    else:
        face_detection = True
        drone_face_detect_button.configure(text='ON', style="ON.TButton")

    drone_face_detect_button.update()

def button_face_center_toggle():
    global face_center, drone_face_button

    if face_center:
        face_center = False
        drone_face_button.configure(text='OFF', style="OFF.TButton")
    else:
        face_center = True
        drone_face_button.configure(text='ON', style="ON.TButton")

    drone_face_button.update()

def button_qr_detect_toggle():
    global qr_detection, drone_qr_detect_button

    if qr_detection:
        qr_detection = False
        drone_qr_detect_button.configure(text='OFF', style="OFF.TButton")
        # text_qr_code.config(state="normal")
        # text_qr_code.delete("1.0", tk.END)
        # text_qr_code.config(state="disabled")
        # text_qr_code.update()
    else:
        qr_detection = True
        drone_qr_detect_button.configure(text='ON', style="ON.TButton")

    drone_qr_detect_button.update()

def button_qr_center_toggle():
    global qr_code_center, drone_qr_center_button

    if qr_code_center:
        qr_code_center = False
        drone_qr_center_button.configure(text='OFF', style="OFF.TButton")
    else:
        qr_code_center = True
        drone_qr_center_button.configure(text='ON', style="ON.TButton")

    drone_qr_center_button.update()

def button_qr_controlled_toggle():
    global qr_controlled, drone_qr_controlled_button

    if qr_controlled:
        qr_controlled = False
        last_command_id = None
        drone_qr_controlled_button.configure(text='OFF', style="OFF.TButton")
    else:
        qr_controlled = True
        drone_qr_controlled_button.configure(text='ON', style="ON.TButton")

    drone_qr_controlled_button.update()

me = tello.Tello()

window = tk.Tk()
window.title("Tello Drone Controller")
frame = ttk.Frame(window)
frame.pack(fill="both", expand=True)

create_lightmode_styles(frame)

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
mp_face_detection = mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5)


# Drone Frame
drone_frame = tk.LabelFrame(frame, text="Drone", background=main_bg_color)
drone_frame.grid(row=0, column=0, sticky="news", padx=10, pady=10)

drone_status_label = ttk.Label(drone_frame, text="Status")
drone_status_label.grid(row=0, column=0)

drone_connect_button = ttk.Button(drone_frame, text="Connect", command=button_connect_drone)
drone_connect_button.grid(row=0, column=1, padx=10, pady=10)

drone_face_detect_label = ttk.Label(drone_frame, text="Face Detection")
drone_face_detect_label.grid(row=1, column=0)

drone_face_detect_button = ttk.Button(drone_frame, text="OFF", style="OFF.TButton", command=button_face_detection_toggle)
drone_face_detect_button.grid(row=1, column=1, padx=10, pady=10)

drone_face_center_label = ttk.Label(drone_frame, text="Center Face")
drone_face_center_label.grid(row=2, column=0)

drone_face_button = ttk.Button(drone_frame, text="OFF", style="OFF.TButton", command=button_face_center_toggle)
drone_face_button.grid(row=2, column=1, padx=10, pady=10)

drone_qr_detect_label = ttk.Label(drone_frame, text="QR-Code Detection")
drone_qr_detect_label.grid(row=3, column=0)

drone_qr_detect_button = ttk.Button(drone_frame, text="OFF", style="OFF.TButton", command=button_qr_detect_toggle)
drone_qr_detect_button.grid(row=3, column=1, padx=10, pady=10)

drone_qr_center_label = ttk.Label(drone_frame, text="Center QR-Code")
drone_qr_center_label.grid(row=4, column=0)

drone_qr_center_button = ttk.Button(drone_frame, text="OFF", style="OFF.TButton", command=button_qr_center_toggle)
drone_qr_center_button.grid(row=4, column=1, padx=10, pady=10)

drone_qr_controlled_label = ttk.Label(drone_frame, text="QR-Code conrolled")
drone_qr_controlled_label.grid(row=5, column=0)

drone_qr_controlled_button = ttk.Button(drone_frame, text="OFF", style="OFF.TButton", command=button_qr_controlled_toggle)
drone_qr_controlled_button.grid(row=5, column=1, padx=10, pady=10)

drone_takeoff_button = ttk.Button(drone_frame, text="Take off", command=lambda: threading.Thread(target=me.takeoff, daemon=True).start())
drone_takeoff_button.grid(row=6, column=0, padx=10, pady=10)

drone_land_button = ttk.Button(drone_frame, text="Land", command=lambda: threading.Thread(target=me.land, daemon=True).start())
drone_land_button.grid(row=6, column=1, padx=10, pady=10)

control_frame = tk.LabelFrame(drone_frame, text="Controls", background=main_bg_color)
control_frame.grid(row=7, column=0, padx=10, pady=10)

# Forward button
control_forward = ttk.Button(control_frame, text="Forward", width=button_width)
control_forward.grid(row=1, column=1, padx=10, pady=10)
control_forward.bind('<ButtonPress-1>', lambda event: start_sending_rc_control_user(0, 20, 0, 0))
control_forward.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Left button
control_left = ttk.Button(control_frame, text="Left", width=button_width)
control_left.grid(row=2, column=0, padx=10, pady=10)
control_left.bind('<ButtonPress-1>', lambda event: start_sending_rc_control_user(-20, 0, 0, 0))
control_left.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Right button
control_right = ttk.Button(control_frame, text="Right", width=button_width)
control_right.grid(row=2, column=2, padx=10, pady=10)
control_right.bind('<ButtonPress-1>', lambda event: start_sending_rc_control_user(20, 0, 0, 0))
control_right.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Back button
control_back = ttk.Button(control_frame, text="Back", width=button_width)
control_back.grid(row=3, column=1, padx=10, pady=10)
control_back.bind('<ButtonPress-1>', lambda event: start_sending_rc_control_user(0, -20, 0, 0))
control_back.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Up button
control_up = ttk.Button(control_frame, text="Up", width=6)
control_up.grid(row=1, column=3, padx=10, pady=10)
control_up.bind('<ButtonPress-1>', lambda event: start_sending_rc_control_user(0, 0, 20, 0))
control_up.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Down button
control_down = ttk.Button(control_frame, text="Down", width=6)
control_down.grid(row=3, column=3, padx=10, pady=10)
control_down.bind('<ButtonPress-1>', lambda event: start_sending_rc_control_user(0, 0, -20, 0))
control_down.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Rotate Left button
control_rotate_left = ttk.Button(control_frame, text="Rotate Left", width=button_width)
control_rotate_left.grid(row=1, column=0, padx=10, pady=10)
control_rotate_left.bind('<ButtonPress-1>', lambda event: start_sending_rc_control_user(0, 0, 0, -30))
control_rotate_left.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())

# Rotate Right button
control_rotate_right = ttk.Button(control_frame, text="Rotate Right", width=button_width)
control_rotate_right.grid(row=1, column=2, padx=10, pady=10)
control_rotate_right.bind('<ButtonPress-1>', lambda event: start_sending_rc_control_user(0, 0, 0, 30))
control_rotate_right.bind('<ButtonRelease-1>', lambda event: stop_sending_rc_control())


# Flip Frame
flip_frame = tk.LabelFrame(drone_frame, text="Flip", background=main_bg_color)
flip_frame.grid(row=7, column=1, padx=10, pady=10)

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
qr_frame = tk.LabelFrame(drone_frame, text="QR Code", background=main_bg_color)
qr_frame.grid(row=8, column=0, sticky="ew", padx=10, pady=10)

qr_label = ttk.Label(qr_frame, text="QR Code Value")
qr_label.grid(row=0, column=0, padx=10, pady=10)

text_qr_code = tk.Text(qr_frame, height=5, width=30, background=main_bg_color)
text_qr_code.grid(row=0, column=1, sticky="ew", padx=10, pady=10)
text_qr_code.config(state="disabled")

# Error Frame
error_frame = tk.LabelFrame(drone_frame, text="Error Log", background=main_bg_color)
error_frame.grid(row=8, column=1, sticky="ew", padx=10, pady=10)

error_log = tk.Text(error_frame, height=5, width=40, background=main_bg_color, fg="red")
error_log.grid(row=0, column=0, padx=10, pady=10)
error_log.config(state="disabled")

# Bild Frame
image_frame = tk.LabelFrame(frame, text="Image", background=main_bg_color)
image_frame.grid(row=0, column=1, sticky="news", padx=10, pady=10)

image_capture = ttk.Label(image_frame, text="Capture")
image_capture.grid(row=0, column=0, padx=10, pady=10, sticky="ew")


# Main GUI Loop
show_default_image()

window.configure(bg=main_bg_color)
window.mainloop()
