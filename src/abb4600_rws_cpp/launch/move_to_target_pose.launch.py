from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    planning_group_arg = DeclareLaunchArgument(
        "planning_group",
        default_value="manipulator"
    )

    velocity_scale_arg = DeclareLaunchArgument(
        "velocity_scale",
        default_value="0.02"
    )

    acceleration_scale_arg = DeclareLaunchArgument(
        "acceleration_scale",
        default_value="0.02"
    )

    planning_time_arg = DeclareLaunchArgument(
        "planning_time",
        default_value="60.0"
    )

    execute_motion_arg = DeclareLaunchArgument(
        "execute_motion",
        default_value="true"
    )

    x_arg = DeclareLaunchArgument("x", default_value="1.594")
    y_arg = DeclareLaunchArgument("y", default_value="0.819")
    z_arg = DeclareLaunchArgument("z", default_value="1.050")

    qx_arg = DeclareLaunchArgument("qx", default_value="0.7574")
    qy_arg = DeclareLaunchArgument("qy", default_value="0.1506")
    qz_arg = DeclareLaunchArgument("qz", default_value="-0.1492")
    qw_arg = DeclareLaunchArgument("qw", default_value="0.6176")

    move_to_target_node = Node(
        package="abb4600_rws_cpp",
        executable="move_to_target_pose",
        name="move_to_target_pose",
        output="screen",
        parameters=[
            {
                "planning_group": LaunchConfiguration("planning_group"),
                "velocity_scale": LaunchConfiguration("velocity_scale"),
                "acceleration_scale": LaunchConfiguration("acceleration_scale"),
                "planning_time": LaunchConfiguration("planning_time"),
                "execute_motion": LaunchConfiguration("execute_motion"),
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
                "qx": LaunchConfiguration("qx"),
                "qy": LaunchConfiguration("qy"),
                "qz": LaunchConfiguration("qz"),
                "qw": LaunchConfiguration("qw"),
            }
        ],
    )

    return LaunchDescription([
        planning_group_arg,
        velocity_scale_arg,
        acceleration_scale_arg,
        planning_time_arg,
        execute_motion_arg,
        x_arg,
        y_arg,
        z_arg,
        qx_arg,
        qy_arg,
        qz_arg,
        qw_arg,
        move_to_target_node,
    ])
