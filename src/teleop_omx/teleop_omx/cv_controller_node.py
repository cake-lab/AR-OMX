# Save this file as: cv_controller_node.py
import pyrealsense2 as rs 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  

import cv2
import mediapipe as mp
import numpy as np
import time

# --- 1. HELPER FUNCTION & PARAMETERS ---
def is_fist_closed(hand_landmarks):
    tip_y = {
        'index': hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP].y,
        'middle': hand_landmarks.landmark[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_TIP].y,
        'ring': hand_landmarks.landmark[mp.solutions.hands.HandLandmark.RING_FINGER_TIP].y,
        'pinky': hand_landmarks.landmark[mp.solutions.hands.HandLandmark.PINKY_TIP].y
    }
    pip_y = {
        'index': hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_PIP].y,
        'middle': hand_landmarks.landmark[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP].y,
        'ring': hand_landmarks.landmark[mp.solutions.hands.HandLandmark.RING_FINGER_PIP].y,
        'pinky': hand_landmarks.landmark[mp.solutions.hands.HandLandmark.PINKY_PIP].y
    }
    # Check if all finger tips are below their PIP joints (closed fist)
    return all(tip_y[finger] > pip_y[finger] for finger in tip_y)

# Velocity mapping parameters
MAX_VEL = 10.0  # meters per second (tune as needed)
ZONE_LOW = 0.4  # lower threshold (40%)
ZONE_HIGH = 0.6  # upper threshold (60%)

# --- 2. CV CONTROLLER NODE ---
class CvControllerNode(Node):
    def __init__(self):
        super().__init__('cv_controller_node')
        # Publisher on '/cmd_twist'
        self.publisher_ = self.create_publisher(Twist, '/cmd_twist', 10)

        # Mediapipe hands pipeline
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_drawing = mp.solutions.drawing_utils

        # OpenCV video capture (default camera)
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')

        # Startup delay
        time.sleep(2)

    def publish_twist(self, y_vel, z_vel):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = float(y_vel)
        twist_msg.linear.z = float(z_vel)
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.publisher_.publish(twist_msg)
        self.get_logger().debug(f'Published Twist: linear.y={y_vel:.3f}, linear.z={z_vel:.3f}')

    def run(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Failed to read frame from camera')
                break

            # Flip horizontally and convert to RGB
            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = self.hands.process(rgb)

            if result.multi_hand_landmarks:
                hand_landmarks = result.multi_hand_landmarks[0]
                # Compute normalized hand center
                coords = [(lm.x, lm.y) for lm in hand_landmarks.landmark]
                cx = np.mean([c[0] for c in coords])
                cy = np.mean([c[1] for c in coords])

                # Piecewise linear zone mapping for Y
                if cx < ZONE_LOW:
                    y_vel = ((ZONE_LOW - cx) / ZONE_LOW) * MAX_VEL  # left zone → +Y
                elif cx > ZONE_HIGH:
                    y_vel = -((cx - ZONE_HIGH) / (1 - ZONE_HIGH)) * MAX_VEL  # right zone → -Y
                else:
                    y_vel = 0.0  # center dead-zone

                # Piecewise linear zone mapping for Z
                if cy < ZONE_LOW:
                    z_vel = -((ZONE_LOW - cy) / ZONE_LOW) * MAX_VEL  # top zone → -Z
                elif cy > ZONE_HIGH:
                    z_vel = ((cy - ZONE_HIGH) / (1 - ZONE_HIGH)) * MAX_VEL  # bottom zone → +Z
                else:
                    z_vel = 0.0  # center dead-zone

                # Stop motion on closed fist
                if is_fist_closed(hand_landmarks):
                    y_vel, z_vel = 0.0, 0.0

                # Publish velocities
                self.publish_twist(y_vel, z_vel)

                # Draw landmarks
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # Overlay velocity text on frame
                vel_text = f"VelY: {y_vel:.2f} m/s, VelZ: {z_vel:.2f} m/s"
                cv2.putText(frame, vel_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2, cv2.LINE_AA)

            # Show the annotated frame
            cv2.imshow('Hand Gesture Control', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Stop before exit
        self.publish_twist(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CvControllerNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
