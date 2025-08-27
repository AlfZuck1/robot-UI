from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('CRSA465')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'CRSA465.urdf')
    controller_yaml = os.path.join(pkg_dir, 'config', 'ros2_control.yaml')
    

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        # micro-ROS Agent (UDP)
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                 'udp4', '--port', '8888', '--dev', '0.0.0.0'],
            output='screen'
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
        ),
        # RosAPI es un complemento de rosbridge que proporciona una API RESTful
        # para interactuar con el sistema ROS
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            output='screen'
        ),
        Node(
            package='CRSA465',
            executable='control_node',
            name='control_node',
            output='screen',
        ),
        # Publicar el modelo del robot en el parámetro 'robot_description'
        # para que se pueda visualizar
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description_content},
                controller_yaml
            ],
            output='screen'
        )
    ])
