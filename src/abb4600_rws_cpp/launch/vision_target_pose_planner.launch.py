from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("planning_group", default_value="manipulator"),
        DeclareLaunchArgument("eef_link", default_value="tool0"),
        DeclareLaunchArgument("reference_frame", default_value=""),

        DeclareLaunchArgument("use_absolute_target", default_value="false"),

        DeclareLaunchArgument("target_x", default_value="1.0"),
        DeclareLaunchArgument("target_y", default_value="0.0"),
        DeclareLaunchArgument("target_z", default_value="1.0"),
        DeclareLaunchArgument("target_qx", default_value="0.0"),
        DeclareLaunchArgument("target_qy", default_value="0.0"),
        DeclareLaunchArgument("target_qz", default_value="0.0"),
        DeclareLaunchArgument("target_qw", default_value="1.0"),

        DeclareLaunchArgument("offset_x", default_value="0.0"),
        DeclareLaunchArgument("offset_y", default_value="0.0"),
        DeclareLaunchArgument("offset_z", default_value="0.05"),

        DeclareLaunchArgument("velocity_scale", default_value="0.10"),
        DeclareLaunchArgument("acceleration_scale", default_value="0.10"),
        DeclareLaunchArgument("planning_time", default_value="60.0"),
        DeclareLaunchArgument("planning_attempts", default_value="10"),

        DeclareLaunchArgument("wrist_singularity_threshold", default_value="0.15"),
        DeclareLaunchArgument("max_joint_jump", default_value="0.50"),

        DeclareLaunchArgument("execute", default_value="false"),
        DeclareLaunchArgument(
            "report_path",
            default_value="/tmp/vision_target_plan_report.txt"
        ),

        Node(
            package="abb4600_rws_cpp",
            executable="vision_target_pose_planner_node",
            name="vision_target_pose_planner",
            output="screen",
            parameters=[{
                "planning_group": LaunchConfiguration("planning_group"),
                "eef_link": LaunchConfiguration("eef_link"),
                "reference_frame": LaunchConfiguration("reference_frame"),

                "use_absolute_target": LaunchConfiguration("use_absolute_target"),

                "target_x": LaunchConfiguration("target_x"),
                "target_y": LaunchConfiguration("target_y"),
                "target_z": LaunchConfiguration("target_z"),
                "target_qx": LaunchConfiguration("target_qx"),
                "target_qy": LaunchConfiguration("target_qy"),
                "target_qz": LaunchConfiguration("target_qz"),
                "target_qw": LaunchConfiguration("target_qw"),

                "offset_x": LaunchConfiguration("offset_x"),
                "offset_y": LaunchConfiguration("offset_y"),
                "offset_z": LaunchConfiguration("offset_z"),

                "velocity_scale": LaunchConfiguration("velocity_scale"),
                "acceleration_scale": LaunchConfiguration("acceleration_scale"),
                "planning_time": LaunchConfiguration("planning_time"),
                "planning_attempts": LaunchConfiguration("planning_attempts"),

                "wrist_singularity_threshold": LaunchConfiguration(
                    "wrist_singularity_threshold"
                ),
                "max_joint_jump": LaunchConfiguration("max_joint_jump"),

                "execute": LaunchConfiguration("execute"),
                "report_path": LaunchConfiguration("report_path"),
            }]
        )
    ])
