import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')
        self.declare_parameter('scan_height', 0.1)
        self.declare_parameter('range_min', 0.0)
        self.declare_parameter('range_max', 100.0)
        self.declare_parameter('scan_frame', 'base_link')
        self.declare_parameter('scan_topic', '/scan')

        self.scan_height = self.get_parameter('scan_height').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_frame = self.get_parameter('scan_frame').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/pointcloud', self.pointcloud_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)

    def pointcloud_callback(self, msg):
        # points = np.array([p[:3] for p in pc2.read_points(
        #     msg, field_names=("x", "y", "z"), skip_nans=True)])
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        points = np.array(points)

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.scan_frame
        scan.angle_min = -np.pi
        scan.angle_max = np.pi
        scan.angle_increment = np.pi / 180  # 1-degree increments
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_readings = int(
            (scan.angle_max - scan.angle_min) / scan.angle_increment)
        ranges = np.full(num_readings, np.inf)

        for point in points:
            x, y, z = point
            if abs(z) > self.scan_height:
                continue
            angle = np.arctan2(y, x)
            dist = np.sqrt(x**2 + y**2)
            if self.range_min < dist < self.range_max:
                index = int((angle - scan.angle_min) / scan.angle_increment)
                if 0 <= index < num_readings:
                    ranges[index] = min(ranges[index], dist)

        scan.ranges = ranges.tolist()
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    # node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
