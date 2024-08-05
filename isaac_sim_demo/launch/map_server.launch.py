from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnStateTransition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    return LaunchDescription([
        # Declare the YAML filename launch argument
        DeclareLaunchArgument(
            'yaml_filename',
            default_value='/home/sags/isaacsim/src/SEI_nav/isaac_sim_demo/maps/third_map2.yaml',
            description='Full path to the map YAML file'
        ),
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': LaunchConfiguration('yaml_filename')}]
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node='map_server',
                goal_state='inactive',
                entities=[
                    ChangeState(
                        lifecycle_node='map_server',
                        transition_id=Transition.TRANSITION_CONFIGURE
                    ),
                ],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node='map_server',
                goal_state='unconfigured',
                entities=[
                    ChangeState(
                        lifecycle_node='map_server',
                        transition_id=Transition.TRANSITION_CONFIGURE
                    ),
                ],
            )
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node='map_server',
                goal_state='configuring',
                entities=[
                    ChangeState(
                        lifecycle_node='map_server',
                        transition_id=Transition.TRANSITION_ACTIVATE
                    ),
                ],
            )
        ),
    ])
