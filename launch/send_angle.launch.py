from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spina_arm_controll',
            executable='serial_controller_node',
            output='screen',
        ),
        Node(
            package='spina_arm_controll',
            executable='angle_send_node',
            output='screen',
        )
    ])