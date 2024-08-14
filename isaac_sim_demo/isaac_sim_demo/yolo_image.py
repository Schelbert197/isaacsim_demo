from enum import Enum, auto
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
# import cv2


class State(Enum):
    """
    The State class.

    Create the states of the node to determine whether there
    is an image to process and whether it should listen.

    """

    AWAITING_IMAGE = auto(),
    PROCESSING_IMAGE = auto()


class YoloImage(Node):
    def __init__(self):
        super().__init__('yolo_image')

        # Load the YOLOv8 model
        self.model = YOLO('yolov8n.pt')

        # Initialize CvBridge
        self.bridge = CvBridge()

        self.image = Image()
        self.state = State.AWAITING_IMAGE

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/front_stereo_camera/left_rgb/image_raw',
            self.image_callback, 10)

        # Publishers
        self.publisher = self.create_publisher(
            Image,
            '/yolo_image',
            10)

        self.timer = self.create_timer(0.2, self.timer_callback)

    def image_callback(self, msg):
        """Callback for the images from the front left camera"""
        self.image = msg

    def process_image(self):
        """Processes the image using YOLOv8"""
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(
                self.image, desired_encoding='bgr8')

            # Run YOLOv8 inference
            results = self.model(cv_image)

            # Process the results (draw bounding boxes on the image)
            for result in results:
                cv_image = result.plot()

            # Convert the processed OpenCV image back to a ROS Image message
            yolo_image_msg = self.bridge.cv2_to_imgmsg(
                cv_image, encoding='bgr8')

            # Publish the processed image
            self.publisher.publish(yolo_image_msg)

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def timer_callback(self):
        """Timer callback with statemachine"""
        if not self.image.data:
            self.state = State.AWAITING_IMAGE
            self.get_logger().info('waiting for image')
        else:
            self.state = State.PROCESSING_IMAGE
            if self.state == State.PROCESSING_IMAGE:
                self.get_logger().debug('processing image')
                self.process_image()
            else:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloImage()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
