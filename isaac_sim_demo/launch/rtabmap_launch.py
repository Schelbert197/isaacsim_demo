from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_ros',
            executable='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_rgbd': True,
                'approx_sync': True,  # Enables approx. time synchronization
                'queue_size': 10,     # Adjust queue size as needed
                'subscribe_imu': True,  # Enables IMU subscription
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('imu', '/imu'),  # Add the IMU topic remapping
            ]
        ),
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'subscribe_depth': True,
                'subscribe_scan': True,
                'subscribe_imu': True,  # Enables IMU subscription
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan', '/scan'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('imu', '/imu'),  # Add the IMU topic remapping
            ]
        ),
        Node(
            package='rtabmap_ros',
            executable='rtabmapviz',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
            }],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('scan', '/scan'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('imu', '/imu'),  # Add the IMU topic remapping
            ]
        )
    ])
