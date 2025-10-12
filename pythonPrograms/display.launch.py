from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import SetLaunchConfiguration
from moveit_configs_utils.launches import generate_demo_launch
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("CRSA465", package_name="crsa465_moveit_config").to_moveit_configs()
    ld = LaunchDescription()
    ld.add_action(
        SetLaunchConfiguration(name="use_rviz", value="True")
    )
    ld.add_action(generate_demo_launch(moveit_config))
        
    return LaunchDescription([
        ld,
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'default_call_service_timeout': 5.0,
                'call_services_in_new_thread': True,
                'send_action_goals_in_new_thread': True
            }],
            output='log',
        ),

        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            output='log'
        ),

        Node(
            package='CRSA465',
            executable='control_node',
            name='control_node',
            output='screen',
        ),
    ])
