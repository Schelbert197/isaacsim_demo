from enum import Enum, auto
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
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

        self.person_detected = False
        self.dectections = 0

        self.marker_array = MarkerArray()
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image,
            '/front_stereo_camera/left_rgb/image_raw',
            self.image_callback, 10)

        # Publishers
        self.image_publisher = self.create_publisher(
            Image,
            '/yolo_image',
            10)
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/person_markers',
            10)

        # Create tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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

            # Set person detected to false before scanning
            self.person_detected = False

            # Run YOLOv8 inference
            results = self.model(cv_image)

            # Process the results (draw bounding boxes on the image)
            for result in results:
                cv_image = result.plot()

                # Check if any of the detected objects is a person
                for box in result.boxes:
                    if box.cls == 0:  # Class 0 in YOLO corresponds to 'person'
                        self.person_detected = True

            # Convert the processed OpenCV image back to a ROS Image message
            yolo_image_msg = self.bridge.cv2_to_imgmsg(
                cv_image, encoding='bgr8')

            # Publish the processed image
            self.image_publisher.publish(yolo_image_msg)

            # Log the detection of a person
            if self.person_detected:
                self.get_logger().info('Person detected in the frame!')

        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def drop_marker(self):
        """Places a marker if the robot detects a human"""
        try:
            self.t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
            qx, qy, qw, qz = self.euler_to_quaternion(
                self.t.transform.rotation.z)
            # Create a marker and add it to the array
            marker = Marker()
            marker.header.stamp = rclpy.time.Time()
            marker.header.frame_id = self.dectections
            marker.ns = 'yolo'
            marker.id = self.dectections
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = self.t.transform.translation.x
            marker.pose.position.y = self.t.transform.translation.y
            marker.pose.position.z = 0.2
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = self.cell_size/2  # Diameter of the cylinder
            marker.scale.y = self.cell_size/8
            marker.scale.z = self.cell_size/8
            marker.color.r = 0.0  # Green color
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            self.marker_array.markers.append(marker)

            self.marker_publisher.publish(self.marker_array)

            self.dectections += 1
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to map: {ex}')

    def euler_to_quaternion(self, yaw=0.0, pitch=0.0, roll=0.0):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
            np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
            np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
            np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return qx, qy, qz, qw

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
