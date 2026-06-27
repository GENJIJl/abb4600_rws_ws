from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rws_ip = LaunchConfiguration("rws_ip")
    rws_port = LaunchConfiguration("rws_port")
    launch_rviz = LaunchConfiguration("launch_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rws_ip",
                default_value="127.0.0.1",
                description="RobotStudio 虚拟控制器的 RWS IP 地址。",
            ),
            DeclareLaunchArgument(
                "rws_port",
                default_value="80",
                description="RobotStudio 虚拟控制器的 RWS HTTP 端口。",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="是否启动带 MoveIt 显示配置的 RViz。",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("abb_bringup"),
                            "launch",
                            "abb_control.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "description_package": "abb_irb4600_support",
                    "description_file": "irb4600_60_205.xacro",
                    "moveit_config_package": "abb_irb4600_60_205_moveit_config",
                    "runtime_config_package": "abb_bringup",
                    "controllers_file": "abb_controllers.yaml",
                    "use_fake_hardware": "false",
                    "rws_ip": rws_ip,
                    "rws_port": rws_port,
                    "launch_rviz": launch_rviz,
                    "initial_joint_controller": "joint_trajectory_controller",
                }.items(),
            ),
        ]
    )
