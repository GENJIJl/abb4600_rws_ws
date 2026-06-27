from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("planning_group", default_value="manipulator"),
        DeclareLaunchArgument("eef_link", default_value="tool0"),

        DeclareLaunchArgument("keep_every", default_value="4"),

        DeclareLaunchArgument("velocity_scale", default_value="0.05"),
        DeclareLaunchArgument("acceleration_scale", default_value="0.05"),
        DeclareLaunchArgument("planning_time", default_value="60.0"),
        DeclareLaunchArgument("planning_attempts", default_value="10"),

        DeclareLaunchArgument("wrist_singularity_threshold", default_value="0.15"),
        DeclareLaunchArgument("max_joint_jump", default_value="0.50"),

        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument("return_to_start", default_value="true"),

        DeclareLaunchArgument(
            "report_path",
            default_value="/tmp/path10_sparse_ompl_report.csv"
        ),

        Node(
            package="abb4600_rws_cpp",
            executable="path10_sparse_ompl_planner_node",
            name="path10_sparse_ompl_planner",
            output="screen",
            parameters=[{
                "planning_group": LaunchConfiguration("planning_group"),
                "eef_link": LaunchConfiguration("eef_link"),

                "keep_every": LaunchConfiguration("keep_every"),

                "velocity_scale": LaunchConfiguration("velocity_scale"),
                "acceleration_scale": LaunchConfiguration("acceleration_scale"),
                "planning_time": LaunchConfiguration("planning_time"),
                "planning_attempts": LaunchConfiguration("planning_attempts"),

                "wrist_singularity_threshold": LaunchConfiguration(
                    "wrist_singularity_threshold"
                ),
                "max_joint_jump": LaunchConfiguration("max_joint_jump"),

                "execute": LaunchConfiguration("execute"),
                "return_to_start": LaunchConfiguration("return_to_start"),
                "report_path": LaunchConfiguration("report_path"),
            }]
        )
    ])
