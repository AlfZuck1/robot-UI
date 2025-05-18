from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('CRSA465'),
        'urdf',
        'CRSA465.urdf'  # o .urdf si no usas xacro
    )

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
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
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    ])
