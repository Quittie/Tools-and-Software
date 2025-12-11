import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist


WINDOW_NAME = "Camera View"


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")

        self.bridge = CvBridge()
        self.last_frame = None
        self.current_label = ""

        # SUBSKRYPCJA KAMERY
        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

        # PUBLIKOWANIE DO TURTLE1
        self.cmd_pub = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10
        )

        # OKNO I CALLBACK MYSZY
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

        self.get_logger().info("CameraNode started.")

    def send_command(self, direction):
        msg = Twist()

        if direction == "FORWARD":
            msg.linear.x = 1.0
        elif direction == "BACKWARD":
            msg.linear.x = -1.0
        else:
            msg.linear.x = 0.0

        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published cmd_vel for {direction}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CVBridge error: {e}")
            return

        # RYSOWANIE NAPISU
        if self.current_label != "":
            cv2.putText(frame, self.current_label, (40, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5,
                        (0, 255, 0), 3)

        self.last_frame = frame
        cv2.imshow(WINDOW_NAME, frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            rclpy.shutdown()

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or self.last_frame is None:
            return

        h, w, _ = self.last_frame.shape
        middle_y = h // 2

        if y < middle_y:
            decision = "FORWARD"
        else:
            decision = "BACKWARD"

        self.current_label = decision
        self.get_logger().info(f"Decision: {decision}")
        self.send_command(decision)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
