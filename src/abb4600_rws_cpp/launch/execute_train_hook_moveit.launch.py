import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    procedure = LaunchConfiguration("procedure")
    velocity_scale = LaunchConfiguration("velocity_scale")
    acceleration_scale = LaunchConfiguration("acceleration_scale")
    planning_time = LaunchConfiguration("planning_time")
    planner_id = LaunchConfiguration("planner_id")
    planning_attempts = LaunchConfiguration("planning_attempts")
    goal_position_tolerance = LaunchConfiguration("goal_position_tolerance")
    goal_orientation_tolerance = LaunchConfiguration("goal_orientation_tolerance")
    trajectory_duration_scale = LaunchConfiguration("trajectory_duration_scale")
    use_approximate_movej_ik = LaunchConfiguration("use_approximate_movej_ik")
    use_exported_movej_jointtargets = LaunchConfiguration("use_exported_movej_jointtargets")
    dry_run = LaunchConfiguration("dry_run")
    validate_trajectory = LaunchConfiguration("validate_trajectory")
    max_joint_step_deg = LaunchConfiguration("max_joint_step_deg")
    eef_step = LaunchConfiguration("eef_step")
    minimum_cartesian_fraction = LaunchConfiguration("minimum_cartesian_fraction")

    support_package = "abb_irb4600_support"
    moveit_config_package = "abb_irb4600_60_205_moveit_config"

    moveit_config = (
        MoveItConfigsBuilder("abb_bringup", package_name=moveit_config_package)
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory(support_package),
                "urdf",
                "irb4600_60_205.xacro",
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(
                get_package_share_directory(moveit_config_package),
                "config",
                "abb_irb4600_60_205.srdf.xacro",
            )
        )
        .robot_description_kinematics(
            file_path=os.path.join(
                get_package_share_directory(moveit_config_package),
                "config",
                "kinematics.yaml",
            )
        )
        .joint_limits(
            file_path=os.path.join(
                get_package_share_directory(moveit_config_package),
                "config",
                "joint_limits.yaml",
            )
        )
        .trajectory_execution(
            file_path=os.path.join(
                get_package_share_directory(moveit_config_package),
                "config",
                "moveit_controllers.yaml",
            ),
            moveit_manage_controllers=False,
        )
        .planning_pipelines()
        .to_moveit_configs()
    )

    return [
        Node(
            package="abb4600_rws_cpp",
            executable="execute_train_hook_moveit_node",
            name="execute_train_hook_moveit_node",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "procedure": procedure,
                    "velocity_scale": velocity_scale,
                    "acceleration_scale": acceleration_scale,
                    "planning_time": planning_time,
                    "planner_id": planner_id,
                    "planning_attempts": planning_attempts,
                    "goal_position_tolerance": goal_position_tolerance,
                    "goal_orientation_tolerance": goal_orientation_tolerance,
                    "trajectory_duration_scale": trajectory_duration_scale,
                    "use_approximate_movej_ik": use_approximate_movej_ik,
                    "use_exported_movej_jointtargets": use_exported_movej_jointtargets,
                    "dry_run": dry_run,
                    "validate_trajectory": validate_trajectory,
                    "max_joint_step_deg": max_joint_step_deg,
                    "eef_step": eef_step,
                    "minimum_cartesian_fraction": minimum_cartesian_fraction,
                },
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("procedure", default_value="guiji_1"),
            DeclareLaunchArgument("velocity_scale", default_value="0.02"),
            DeclareLaunchArgument("acceleration_scale", default_value="0.02"),
            DeclareLaunchArgument("planning_time", default_value="30.0"),
            DeclareLaunchArgument("planner_id", default_value="RRTConnectkConfigDefault"),
            DeclareLaunchArgument("planning_attempts", default_value="10"),
            DeclareLaunchArgument("goal_position_tolerance", default_value="0.005"),
            DeclareLaunchArgument("goal_orientation_tolerance", default_value="0.05"),
            DeclareLaunchArgument("trajectory_duration_scale", default_value="1.0"),
            DeclareLaunchArgument("use_approximate_movej_ik", default_value="true"),
            DeclareLaunchArgument("use_exported_movej_jointtargets", default_value="true"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("validate_trajectory", default_value="true"),
            DeclareLaunchArgument("max_joint_step_deg", default_value="90.0"),
            DeclareLaunchArgument("eef_step", default_value="0.02"),
            DeclareLaunchArgument("minimum_cartesian_fraction", default_value="0.90"),
            OpaqueFunction(function=launch_setup),
        ]
    )
