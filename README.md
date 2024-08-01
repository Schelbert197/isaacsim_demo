# IsaacSim Demo

This package exists to showcase some of the things that roboticists can do to develop robots using the Nvidia IsaacSim platform.

## Prerequisites
There are some assumptions that this package expects regarding the information bridged from IsaacSim. The .usd files included provide the following over the ros2 bridge:
- `/odom`
- `/pointcloud` which is then converted from PointCloud2 to LaserScan on `/scan` using the points_to_scan node (to use slam_toolbox)
- `/front_stereo_camera/left_rgb/camerainfo`
- `/front_stereo_camera/left_rgb/image_raw`
- `/front_stereo_camera/right_rgb/camerainfo`
- `/front_stereo_camera/right_rgb/image_raw`
- `/cmd_vel` (this is bridged as an input to which IsaacSim is subscribed)
- `/imu/data`

## How to use
1. Start your simulation in IsaacSim and confirm that the topics are being bridged from the simulator.
2. Open a terminal instance and source the workspace containing the package.
3. Run the command `ros2 launch isaac_sim_demo start_mapping.launch.xml`.
    - This will launch Rviz2 with the slam_toolbox settings established
    - This will launch the flattener node points_to_scan for slam_toolbox to use
4. Run the command `ros2 run teleop_twist_keyboard teleop_twist_keyboard` to control the robot and create a 2D OccupancyGrid map that can be seen in Rviz2. 
5. To save the map you can either use the slam_toolbox Rviz2 plugin, or run the command `ros2 run nav2_map_server map_saver_cli -f /path/to/your/file/name` where the path portion is replaced with the location and name you want to provide for your new map.