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
            package='joy',
            executable='joy_node',
            output='screen',
        ),
        Node(
            package='spina_arm_controll',
            executable='joy_spina_node',
            output='screen',
        )
    ])