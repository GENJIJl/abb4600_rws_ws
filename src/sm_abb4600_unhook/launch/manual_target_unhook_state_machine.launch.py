from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sm_abb4600_unhook',
            executable='manual_target_unhook_state_machine_node',
            name='manual_target_unhook_state_machine',
            output='screen',
        ),
    ])
