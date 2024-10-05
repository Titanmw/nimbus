import time
from djitellopy import tello
import cv2
import mediapipe as mp

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils
me = tello.Tello()
me.connect()
me.streamon()
target = False
key = False

with mp_face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.5) as face_detection:
    while True:
        image = me.get_frame_read().frame
        image.flags.writeable = True
        image = cv2.flip(image, 1)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.putText(image, f'Battery: {me.get_battery()}%', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        if target:
            image = cv2.putText(image, f'AutoAim: On', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            image = cv2.putText(image, f'AutoAim: Off', (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        results = face_detection.process(image)
        if results.detections:
            for detection in results.detections:
                mp_drawing.draw_detection(image, detection)
                if me.is_flying and target:
                    nose = (0.5 - mp_face_detection.get_key_point(detection, mp_face_detection.FaceKeyPoint.NOSE_TIP).x) * 2
                    try:
                        if nose < 0.2:
                            me.rotate_clockwise(int(nose * 40))
                        elif nose > 0.2:
                            me.rotate_counter_clockwise(int((nose * -1) * 40))
                    except:
                        print('error at aiming')
                    continue
        cv2.imshow('MediaPipe Face Detection', image)

        try:
            key = cv2.waitKey(100)
            if me.is_flying:
                if key == ord('w'):
                    me.move_forward(50)
                if key == ord('s'):
                    me.move_back(50)
                if key == ord('a'):
                    me.move_left(50)
                if key == ord('d'):
                    me.move_right(50)
                if key == ord('q'):
                    me.rotate_counter_clockwise(45)
                if key == ord('e'):
                    me.rotate_clockwise(45)
                if key == ord('l'):
                    me.land()
                if key == ord('o'):
                    me.move_up(50)
                if key == ord('k'):
                    me.move_down(50)
            else:
                if key == ord('t'):
                    me.takeoff()
        except:
            print('error at moving')
        if key == ord('x'):
            me.end()
            cv2.destroyAllWindows()
            exit(0)
        if key == ord('z'):
            if target:
                target = False
            else:
                target = True
        time.sleep(0.05)
