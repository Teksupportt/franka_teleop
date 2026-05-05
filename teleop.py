import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import cv2
import mediapipe as mp
import numpy as np
from ikpy.chain import Chain
import threading
import time

chain = Chain.from_urdf_file(
    "/home/teksupportt/ros/panda.urdf",
    base_elements=["panda_link0"],
    last_link_vector=[0, 0, 0.1],
    base_element_type="link"
)

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

class SmoothIKTeleop(Node):
    def __init__(self):
        super().__init__('smooth_ik_teleop')
        self.pub = self.create_publisher(JointState, '/joint_command', 10)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.hands = mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5,
            model_complexity=0
        )

        self.prev_target     = np.array([0.4, 0.0, 0.5])
        self.prev_joints     = np.zeros(7)
        self.alpha           = 0.4
        self.max_joint_step  = 0.15
        self.gripper_opening = 0.08   # start fully open

        self.latest_frame     = None
        self.latest_landmarks = None
        self.latest_joints    = None
        self.hand_detected    = False
        self.current_target   = np.array([0.4, 0.0, 0.5])
        self.frame_lock       = threading.Lock()
        self.landmark_lock    = threading.Lock()
        self.joints_lock      = threading.Lock()
        self.running          = True

        threading.Thread(target=self.camera_loop,    daemon=True).start()
        threading.Thread(target=self.mediapipe_loop, daemon=True).start()
        threading.Thread(target=self.ik_loop,        daemon=True).start()

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info('SmoothIKTeleop started — waiting for hand...')

    def camera_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            frame = cv2.flip(frame, 1)
            with self.frame_lock:
                self.latest_frame = frame
            time.sleep(0.01)

    def mediapipe_loop(self):
        while self.running:
            frame_copy = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame_copy = self.latest_frame.copy()

            if frame_copy is None:
                time.sleep(0.01)
                continue

            rgb = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2RGB)
            rgb.flags.writeable = False
            result = self.hands.process(rgb)
            rgb.flags.writeable = True

            landmarks = None
            detected  = False
            if result.multi_hand_landmarks:
                landmarks = result.multi_hand_landmarks[0]
                detected  = True
                mp_draw.draw_landmarks(frame_copy, landmarks, mp_hands.HAND_CONNECTIONS)

            with self.frame_lock:
                if detected:
                    self.latest_frame = frame_copy

            with self.landmark_lock:
                self.latest_landmarks = landmarks
                self.hand_detected    = detected

            time.sleep(0.033)

    def get_gripper_opening(self, hand_landmarks):
        """Returns 0.0 = fully closed, 1.0 = fully open"""
        fingertips = [4,  8,  12, 16, 20]
        bases      = [2,  5,  9,  13, 17]

        distances = []
        for tip, base in zip(fingertips, bases):
            tip_pos  = np.array([hand_landmarks.landmark[tip].x,
                                 hand_landmarks.landmark[tip].y])
            base_pos = np.array([hand_landmarks.landmark[base].x,
                                 hand_landmarks.landmark[base].y])
            distances.append(np.linalg.norm(tip_pos - base_pos))

        avg_distance = np.mean(distances)
        min_dist = 0.05
        max_dist = 0.20
        openness = (avg_distance - min_dist) / (max_dist - min_dist)
        return float(np.clip(openness, 0.0, 1.0))

    def ik_loop(self):
        initial_position = [0] * len(chain.links)
        initial_position[1:8] = [0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4]

        while self.running:
            with self.landmark_lock:
                hand = self.latest_landmarks

            if hand is None:
                time.sleep(0.01)
                continue

            x = hand.landmark[0].x
            y = hand.landmark[0].y

            target = np.array([
                0.3 + 0.6 * (x - 0.5),
                0.0 + 0.8 * (y - 0.5),
                0.5
            ])

            target = self.low_pass(self.prev_target, target)
            self.prev_target    = target
            self.current_target = target

            try:
                ik = chain.inverse_kinematics(
                    target_position=target,
                    target_orientation=None,
                    orientation_mode=None,
                    initial_position=initial_position
                )

                joints = np.array(ik[1:8])
                joints = self.limit_step(self.prev_joints, joints)
                self.prev_joints = joints
                initial_position[1:8] = joints.tolist()

                with self.joints_lock:
                    self.latest_joints = joints.copy()

            except Exception as e:
                self.get_logger().warn(f'IK failed: {e}')

            # Gripper calculation
            openness             = self.get_gripper_opening(hand)
            gripper_pos          = openness * 0.08
            self.gripper_opening = self.low_pass(self.gripper_opening, gripper_pos)

            time.sleep(0.033)

    def low_pass(self, prev, new):
        return (1 - self.alpha) * prev + self.alpha * new

    def limit_step(self, prev, target):
        delta = np.clip(target - prev, -self.max_joint_step, self.max_joint_step)
        return prev + delta

    def loop(self):
        with self.frame_lock:
            frame = self.latest_frame
        if frame is None:
            return

        with self.landmark_lock:
            detected = self.hand_detected

        with self.joints_lock:
            joints = self.latest_joints

        display = frame.copy()

        # Hand detected indicator
        if detected:
            cv2.circle(display, (30, 30), 15, (0, 255, 0), -1)
            cv2.putText(display, "HAND DETECTED", (55, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.circle(display, (30, 30), 15, (0, 0, 255), -1)
            cv2.putText(display, "NO HAND", (55, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Gripper bar
        openness_display = self.gripper_opening / 0.08
        bar_width = int(200 * openness_display)
        cv2.rectangle(display, (10, 60),  (210, 85), (50, 50, 50), -1)
        cv2.rectangle(display, (10, 60),  (10 + bar_width, 85), (0, 200, 255), -1)
        cv2.putText(display, f"Gripper: {int(openness_display * 100)}%",
                    (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        # Target XYZ
        cv2.putText(display,
                    f"Target: [{self.current_target[0]:.2f}, "
                    f"{self.current_target[1]:.2f}, {self.current_target[2]:.2f}]",
                    (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)

        # Joints + publish arm and gripper together
        if joints is not None:
            joints_deg = np.degrees(joints)
            cv2.putText(display,
                        f"J: {' '.join([f'{j:.0f}' for j in joints_deg])}",
                        (10, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [
                'panda_joint1', 'panda_joint2', 'panda_joint3',
                'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
                'panda_finger_joint1', 'panda_finger_joint2'
            ]
            msg.position = joints.tolist() + [self.gripper_opening, self.gripper_opening]
            msg.velocity = []
            msg.effort   = []
            self.pub.publish(msg)

        cv2.imshow("Smooth IK Teleop", display)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = SmoothIKTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()