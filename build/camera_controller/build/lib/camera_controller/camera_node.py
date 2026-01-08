#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import cv2
import numpy as np
import time

WINDOW_NAME = "Camera View"


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.bridge = CvBridge()
        self.last_frame = None
        self.current_label = ""
        self.last_cmd = None          # "FORWARD"/"BACKWARD"/"STOP"
        self.last_pub_time = 0.0      # do rate limit

        # --- PARAM: control mode ---
        # "mouse" albo "aruco"
        self.declare_parameter("control_mode", "aruco")
        self.control_mode = self.get_parameter("control_mode").get_parameter_value().string_value
        self.get_logger().info(f"control_mode = {self.control_mode}")

        # --- SUB: camera ---
        self.image_sub = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

        # --- PUB: turtle cmd_vel ---
        self.cmd_pub = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10
        )

        # --- OpenCV window + mouse callback ---
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

        # --- ArUco setup (z fallbackiem) ---
        self.aruco_ok = hasattr(cv2, "aruco")
        if self.aruco_ok:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            try:
                # nowsze OpenCV
                self.aruco_params = cv2.aruco.DetectorParameters()
                self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                self.get_logger().info("ArUco: using cv2.aruco.ArucoDetector (new API)")
            except Exception:
                # starsze OpenCV
                self.aruco_params = cv2.aruco.DetectorParameters_create()
                self.aruco_detector = None
                self.get_logger().info("ArUco: using detectMarkers (old API)")
        else:
            self.get_logger().warn("OpenCV has no aruco module. Install opencv-contrib-python or proper system OpenCV.")

        self.get_logger().info("Camera node started.")

    # -------------------------
    # CMD publishing (safe)
    # -------------------------
    def publish_cmd(self, direction: str):
        """
        direction: "FORWARD", "BACKWARD", "STOP"
        """
        # rate limit: max 10 Hz + publish tylko jak zmiana
        now = time.time()
        if direction == self.last_cmd and (now - self.last_pub_time) < 0.1:
            return

        msg = Twist()
        if direction == "FORWARD":
            msg.linear.x = 1.0
        elif direction == "BACKWARD":
            msg.linear.x = -1.0
        else:
            msg.linear.x = 0.0

        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

        self.last_cmd = direction
        self.last_pub_time = now

    # -------------------------
    # Image callback
    # -------------------------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        self.last_frame = frame

        # --- CONTROL: ARUCO MODE ---
        if self.control_mode == "aruco":
            self.handle_aruco(frame)

        # --- draw label if exists ---
        if self.current_label:
            cv2.putText(
                frame,
                self.current_label,
                (40, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.2,
                (0, 255, 0),
                3
            )

        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            rclpy.shutdown()

    # -------------------------
    # ArUco logic
    # -------------------------
    def handle_aruco(self, frame):
        if not self.aruco_ok:
            self.current_label = "ARUCO: NOT AVAILABLE"
            self.publish_cmd("STOP")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = frame.shape[:2]
        mid_y = h // 2

        # draw image center line
        cv2.line(frame, (0, mid_y), (w, mid_y), (255, 255, 0), 2)

        # detect markers (new API or old)
        if self.aruco_detector is not None:
            corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or len(ids) == 0:
            self.current_label = "ARUCO: NOT FOUND"
            self.publish_cmd("STOP")
            return

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # use first marker
        c = corners[0][0]  # 4 corners
        cx = int(c[:, 0].mean())
        cy = int(c[:, 1].mean())

        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

        if cy < mid_y:
            self.current_label = "ARUCO: FORWARD"
            self.publish_cmd("FORWARD")
        else:
            self.current_label = "ARUCO: BACKWARD"
            self.publish_cmd("BACKWARD")

    # -------------------------
    # Mouse callback
    # -------------------------
    def mouse_callback(self, event, x, y, flags, param):
        if self.control_mode != "mouse":
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.last_frame is None:
            return

        h, w = self.last_frame.shape[:2]
        mid_y = h // 2

        if y < mid_y:
            self.current_label = "MOUSE: FORWARD"
            self.publish_cmd("FORWARD")
        else:
            self.current_label = "MOUSE: BACKWARD"
            self.publish_cmd("BACKWARD")

        self.get_logger().info(self.current_label)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
