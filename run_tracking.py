import threading
import queue
import cv2
import time
import numpy as np

import robosuite as suite

import mediapipe.python.solutions.hands as mp_hands
import mediapipe.python.solutions.drawing_utils as mp_drawing
import mediapipe.python.solutions.drawing_styles as mp_drawing_styles
from mediapipe.python.solutions.hands_connections import HAND_CONNECTIONS

from hand_tracking import calibrate_index, get_params
from hand_policy import HandTrackingWithPID


command_queue = queue.Queue()


def hand_tracking():
    cap = cv2.VideoCapture(index=0)

    custom_style = mp_drawing_styles.get_default_pose_landmarks_style()
    custom_connections = list(HAND_CONNECTIONS)

    excluded_landmarks = [
        mp_hands.HandLandmark.WRIST,
        mp_hands.HandLandmark.THUMB_CMC,
        mp_hands.HandLandmark.THUMB_MCP,
        mp_hands.HandLandmark.THUMB_IP,
        mp_hands.HandLandmark.INDEX_FINGER_MCP,
        mp_hands.HandLandmark.INDEX_FINGER_PIP,
        mp_hands.HandLandmark.INDEX_FINGER_DIP,
        mp_hands.HandLandmark.MIDDLE_FINGER_MCP,
        mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
        mp_hands.HandLandmark.MIDDLE_FINGER_DIP,
        mp_hands.HandLandmark.RING_FINGER_MCP,
        mp_hands.HandLandmark.RING_FINGER_PIP,
        mp_hands.HandLandmark.RING_FINGER_DIP,
        mp_hands.HandLandmark.RING_FINGER_TIP,
        mp_hands.HandLandmark.PINKY_MCP,
        mp_hands.HandLandmark.PINKY_PIP,
        mp_hands.HandLandmark.PINKY_DIP,
        mp_hands.HandLandmark.PINKY_TIP
    ]

    for landmark in excluded_landmarks:
        # change the way the landmarks are drawn
        custom_style[landmark] = mp_drawing.DrawingSpec(color=(255,255,0), thickness=0) 
        custom_connections = [connection_tuple for connection_tuple in custom_connections 
                                if landmark.value not in connection_tuple]
        
    custom_style[mp_hands.HandLandmark.INDEX_FINGER_TIP] = mp_drawing.DrawingSpec(color=(0,0,255), thickness=5)
    custom_style[mp_hands.HandLandmark.THUMB_TIP] = mp_drawing.DrawingSpec(color=(0,0,255), thickness=5)
    custom_style[mp_hands.HandLandmark.MIDDLE_FINGER_TIP] = mp_drawing.DrawingSpec(color=(0,0,255), thickness=5)


    with mp_hands.Hands(
        model_complexity=0,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    ) as hands:
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                continue

            # check frame for hands
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = hands.process(frame_rgb)

            image = cv2.resize(frame, (1000, 750))
            height, width, _ = image.shape

            circle_center = (int(width * 0.3), int(height*0.25))
            radius = 20
            color = (0, 255, 0)
            thickness = -1
            cv2.circle(image, circle_center, radius, color, thickness)

            # draw detected hand landmarks on frame
            # draw the hand annotations on the image
            # RH @ index 1 
            # LH @ index 0
            if results.multi_hand_landmarks:
                for hand_landmarks, hand_handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    label = hand_handedness.classification[0].label

                    mp_drawing.draw_landmarks(
                        image=image,
                        landmark_list=hand_landmarks,
                        connections=custom_connections,
                        landmark_drawing_spec=custom_style,
                    )

                    
                    if label == "Left":
                        calibrated = calibrate_index(circle_center, radius, hand_landmarks.landmark[8], width, height)

                        thumb_rhs = hand_landmarks.landmark[4]
                        index_rhs = hand_landmarks.landmark[8]
                        middle_rhs = hand_landmarks.landmark[12]

                        params_rhs = get_params(index_rhs, thumb_rhs, middle_rhs)
                        params_rhs.append(calibrated)
                        params_rhs.append(False)
                        # print(params_rhs)
                        command_queue.put(params_rhs)


            cv2.imshow("Hand Tracking", cv2.flip(image, 1))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cap.release()


def robot_sim():
    
    env = suite.make(
        env_name="Stack",
        robots="Panda",
        has_renderer=True,
        has_offscreen_renderer=False,
        use_camera_obs=False,
        horizon=6000,
        control_freq=20
    )

    success_rate = 0
    print("Waiting to calibrate - place right index finger in green circle")
    # reset the environment
    for _ in range(5):
        obs = env.reset()

        policy = HandTrackingWithPID(obs['robot0_eef_pos']) 
        while True:

            try:
                params = command_queue.get_nowait()

            except queue.Empty:
                params = None

            action = policy.get_action(params, obs['robot0_eef_pos'])
            obs, reward, done, info = env.step(action)  # take action in the environment
            env.render()  # render on display
            if reward == 1.0:
                success_rate += 1
                break


    success_rate /= 5.0
    print('success rate:', success_rate)


threading.Thread(target=hand_tracking, daemon=True).start()
robot_sim()