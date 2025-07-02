import cv2
import mediapipe as mp
import math
import numpy as np
# import controller
import time  # For delay control
from cvfpscalc import CvFpsCalc
from collections import deque

# IF WEBCAM INDEX OUT OF RANGE, TRY CHANGING VIDEOCAPTURE(0) -> VIDEOCAPTURE(1)
# LIBRARIES:
# OPENCV-PYTHON
# MEDIAPIPE

# ADJUST HAND BEND VALUES BASED ON THE HIGHEST AND LOWEST VALUES ON PRINTED VALUES IN TERMINAL
handBendMax = 2.5
handBendMin = 1.5


class InputSmoother:
    def __init__(self, window_size=10):
        self.window_size = window_size
        self.inputs = deque(maxlen=window_size)

    def add_input(self, value):
        self.inputs.append(value)
        return self.get_smoothed_value()

    def get_smoothed_value(self):
        if not self.inputs:
            return 0
        return sum(self.inputs) / len(self.inputs)
    

# Utility functions
def calculate_distance(point1, point2):
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def map_value(value, from_low, from_high, to_low, to_high):
    mapped = (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
    return max(to_low, min(to_high, mapped))  # Ensure value stays within bounds

# MediaPipe Initialization
mp_draw = mp.solutions.drawing_utils
mp_hand = mp.solutions.hands
hands = mp_hand.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5, max_num_hands=1)
video = cv2.VideoCapture(1)

FRAME_CENTER_X = int(video.get(3) / 2)
FRAME_CENTER_Y = int(video.get(4) / 2)

# Servo mappings
SERVO_LATERAL = 1
SERVO_VERTICAL_1 = 2
SERVO_VERTICAL_2 = 3
SERVO_VERTICAL_3 = 4
SERVO_CLAW = 6  # Pin 11 for claw control

# Initial servo positions
servo_angles = {
    SERVO_LATERAL: 0,
    SERVO_VERTICAL_1: 0,
    SERVO_VERTICAL_2: 0,
    4: 0,
    SERVO_VERTICAL_3: 0,
}




# Gradual Calibration to 90° at Startup
calibration_speed = 1
while any(angle != 90 for angle in servo_angles.values()):
    for servo in servo_angles:
        if servo_angles[servo] < 90:
            servo_angles[servo] += calibration_speed
        elif servo_angles[servo] > 90:
            servo_angles[servo] -= calibration_speed

        # controller.set_servo_angle(servo, servo_angles[servo])

    time.sleep(0.02)

# Sensitivity controls
sensitivity = 90
cvFpsCalc = CvFpsCalc(buffer_len=20)
firstJointSmoother = InputSmoother()
secondJointSmoother = InputSmoother()

camWidth = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
camHeight = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))

while True:
    fps = cvFpsCalc.get()
    ret, image = video.read()
    if not ret:
        break

    image = cv2.flip(image, 1)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = hands.process(image_rgb)

    # Draw center point
    cv2.circle(image, (FRAME_CENTER_X, FRAME_CENTER_Y), 5, (0, 0, 255), -1)

    if results.multi_hand_landmarks:
        hand_landmark = results.multi_hand_landmarks[0]
        lmList = [[id, int(lm.x * image.shape[1]), int(lm.y * image.shape[0])] for id, lm in enumerate(hand_landmark.landmark)]

        mp_draw.draw_landmarks(image, hand_landmark, mp_hand.HAND_CONNECTIONS)

        if lmList:
            # Code 1's tracking logic
            hand_x, hand_y = lmList[9][1], lmList[9][2]
            lateral_offset = (FRAME_CENTER_X - hand_x) / sensitivity
            vertical_offset = (hand_y - FRAME_CENTER_Y) / sensitivity

            servo_angles[SERVO_LATERAL] -= lateral_offset
            # servo_angles[SERVO_VERTICAL_1] -= vertical_offset
            # servo_angles[SERVO_VERTICAL_2] += vertical_offset
            # servo_angles[SERVO_VERTICAL_3] -= vertical_offset
            
            # Code 2's tracking logic (Claw control)
            thumb_tip = [int(lmList[4][1]/camWidth * 100), int(lmList[4][2]/camHeight * 100)]
            
            middle_finger_tip = [int(lmList[12][1]/camWidth * 100), int(lmList[12][2]/camHeight * 100)]
            wrist = [int(lmList[0][1]/camWidth * 100), int(lmList[0][2]/camHeight * 100)]
            index_mcp = [int(lmList[5][1]/camWidth * 100), int(lmList[5][2]/camHeight * 100)]
            # print(index_mcp)
            center_Hand = [int(lmList[9][1]/camWidth * 100), int(lmList[9][2]/camHeight * 100)]
            thumb_cmc = [int(lmList[1][1]/camWidth * 100), int(lmList[1][2]/camHeight * 100)]
            center_HandYAxis = center_Hand[1]

            # Finding Y-Axis
            center_HandMap = round(map_value(center_HandYAxis, 100, 0, 0, 80))
            # print(center_HandYAxis)
            smoothed_distance2 = secondJointSmoother.add_input(center_HandMap)
            cv2.putText(image, f"Y Axis: {smoothed_distance2}", (10, 150), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 200, 200), 2, cv2.LINE_AA)
            # controller.set_servo_angle(SERVO_VERTICAL_2, smoothed_distance2)
            # controller.set_servo_angle(SERVO_VERTICAL_1, 70)


           # Finding Hand Bend
            reference_distance = calculate_distance((wrist[0], wrist[1]), (thumb_cmc[0], thumb_cmc[1]))
            hand_bend = calculate_distance((wrist[0], wrist[1]), (index_mcp[0], index_mcp[1]))/reference_distance
            print("hand bend raw value: " + str(hand_bend))
            distanceMapped = round(map_value(hand_bend, handBendMin, handBendMax, 90, 180), 2)
            smoothed_distance = firstJointSmoother.add_input(distanceMapped)
            # controller.set_servo_angle(SERVO_VERTICAL_3, smoothed_distance)
            cv2.putText(image, f"Hand Bend: {smoothed_distance}", (10, 120), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 200, 200), 2, cv2.LINE_AA)

            # Wrist circle
            cv2.circle(image, (lmList[0][1], lmList[0][2]), 7, (100, 255, 100), 3)

            
            if reference_distance > 0:
                normalized_distance = calculate_distance((thumb_tip[0], thumb_tip[1]), (middle_finger_tip[0], middle_finger_tip[1])) / reference_distance
                mapped_distance = map_value(normalized_distance, 0.1, 1.5, 0, 100)
            else:
                mapped_distance = 0

            cv2.line(image, (lmList[0][1], lmList[0][2]), (lmList[5][1], lmList[5][2]), (0, 255, 0), 2)
            # cv2.putText(image, f"Angle: {mapped_distance}", (10, 60), cv2.FONT_HERSHEY_COMPLEX, 1, (100, 255, 100), 2, cv2.LINE_AA)
            # controller.set_servo_angle(SERVO_CLAW, mapped_distance)

    # Clamp angles between 0 and 180
    for servo in servo_angles:
        servo_angles[servo] = max(0, min(180, servo_angles[servo]))
        # controller.set_servo_angle(SERVO_LATERAL, servo_angles[SERVO_LATERAL])

    # Display debug info
    cv2.putText(image, f"Lateral: {servo_angles[SERVO_LATERAL]:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    cv2.putText(image, f"FPS: {fps}", (10, 90), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 100, 100), 2, cv2.LINE_AA)
    

    cv2.imshow("Frame", image)
    if cv2.waitKey(1) == 27:
        break

video.release()
cv2.destroyAllWindows()
# controller.cleanup()
