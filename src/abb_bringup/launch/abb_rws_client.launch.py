from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 这些参数用于连接 ABB 控制器的 RWS HTTP 服务。
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    robot_nickname = LaunchConfiguration("robot_nickname")
    polling_rate = LaunchConfiguration("polling_rate")
    no_connection_timeout = LaunchConfiguration("no_connection_timeout")

    declared_arguments = []

    # robot_ip 是 RobotStudio 或真实控制器的 IP 地址。
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="None",
            description="IP address to the robot controller's RWS server",
        )
    )

    # robot_port 是 RWS 服务端口，默认通常是 80。
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="80",
            description="Port number of the robot controller's RWS server",
        )
    )

    # robot_nickname 是用户自定义的机器人标识，不参与底层通信协议。
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_nickname",
            default_value="",
            description="Arbitrary user nickname/identifier for the robot controller",
        )
    )

    # no_connection_timeout 控制初始化时是否可以无限等待控制器连接。
    declared_arguments.append(
        DeclareLaunchArgument(
            "no_connection_timeout",
            default_value="false",
            description="Specifies whether the node is allowed to wait indefinitely \
            for the robot controller during initialization.",
        )
    )

    # polling_rate 控制 RWS 状态轮询频率。
    declared_arguments.append(
        DeclareLaunchArgument(
            "polling_rate",
            default_value="5.0",
            description="The frequency [Hz] at which the controller state is collected.",
        )
    )

    # rws_client 节点提供 RWS 服务调用和状态发布功能。
    node = Node(
        package="abb_rws_client",
        executable="rws_client",
        name="rws_client",
        output="screen",
        parameters=[
            {"robot_ip": robot_ip},
            {"robot_port": robot_port},
            {"robot_nickname": robot_nickname},
            {"polling_rate": polling_rate},
            {"no_connection_timeout": no_connection_timeout},
        ],
    )
    # 返回参数声明和 RWS 客户端节点。
    return LaunchDescription(declared_arguments + [node])
