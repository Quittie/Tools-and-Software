import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


WINDOW_NAME = 'Camera view'


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()
        self.last_frame = None
        self.current_label = ""   # <- napis, który pokażemy na ekranie

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

        self.get_logger().info('CameraNode with on-screen labels started.')

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # Jeśli mamy tekst do wyświetlenia → nakładamy go na obraz
        if self.current_label != "":
            cv2.putText(frame, self.current_label, (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        self.last_frame = frame
        cv2.imshow(WINDOW_NAME, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('q pressed, shutting down node...')
            rclpy.shutdown()

    def mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.last_frame is None:
            return

        h, w, _ = self.last_frame.shape
        middle_y = h // 2

        if y < middle_y:
            decision = "FORWARD"
        else:
            decision = "BACKWARD"

        self.current_label = decision  # <- wyświetlamy napis
        self.get_logger().info(f"Decision: {decision}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
