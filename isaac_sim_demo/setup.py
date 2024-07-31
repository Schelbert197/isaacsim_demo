from setuptools import find_packages, setup

package_name = 'isaac_sim_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         [  # 'launch/saltbot_visual.launch.xml',
             #   'launch/localization_launch.py',
            #   'launch/navigation_launch.py',
            #   'launch/jackal_nav.launch.py',
            #   'launch/start_3d_slam.launch.xml',
            #   'launch/velodyne.launch.py',
            #   'launch/filtered_velodyne.launch.py',
            'launch/rtabmap_launch.py',
            'launch/rviz_launch.py']),

        ('share/' + package_name + '/config',
         ['config/isaacsim.rviz',
          'config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sags',
    maintainer_email='srikanthschelbert2024@u.northwestern.edu',
    description='A package designed to showcase the simulation of a diff \
        drive robot in IsaacSim using ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'points_to_scan = isaac_sim_demo.pointcloud_to_laserscan:main'
        ],
    },
)
