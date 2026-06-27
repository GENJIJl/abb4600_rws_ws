from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessIO
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # declared_arguments 用来集中声明 ros2 launch 可以从命令行传入的参数。
    declared_arguments = []

    # runtime_config_package 指向控制器 YAML 所在的 ROS 2 包。
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="abb_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    # controllers_file 是 controller_manager 读取的控制器配置文件名。
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="abb_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    # description_package 指向机器人 URDF/Xacro 所在的 support 包。
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    # moveit_config_package 用来给 RViz 找 MoveIt 的显示配置。
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="",
            description="MoveIt configuration package for the robot, e.g. abb_irb1200_5_90_moveit_config",
        )
    )
    # description_file 是具体机器人型号的 Xacro 文件。
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="",
            description="URDF/XACRO description file with the robot, e.g. irb1200_5_90.xacro",
        )
    )
    # prefix 用于多机器人场景，给 joint/link 名称加前缀。
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed then also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    # use_fake_hardware 决定 ros2_control 使用假硬件还是真实 ABB 硬件接口。
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    # rws_ip 是 RobotStudio 或真实控制器的 RWS 地址。
    declared_arguments.append(
        DeclareLaunchArgument(
            "rws_ip",
            default_value="None",
            description="IP of RWS computer. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    # rws_port 是 RWS HTTP 服务端口，默认通常是 80。
    declared_arguments.append(
        DeclareLaunchArgument(
            "rws_port",
            default_value="80",
            description="Port at which RWS can be found. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    # configure_via_rws 决定硬件接口是否通过 RWS 查询机器人控制器描述。
    declared_arguments.append(
        DeclareLaunchArgument(
            "configure_via_rws",
            default_value="true",
            description="If false, the robot description will be generate from joint information \
            in the ros2_control xacro. Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    # fake_sensor_commands 只在 fake hardware 模式下使用。
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    # initial_joint_controller 是启动时要激活的主运动控制器。
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    # start_initial_joint_controller=false 时只启动状态链路，不激活轨迹控制器。
    # 这适合先打开 EGM 并保持当前位姿，确认 /joint_states 稳定后再手动激活控制器。
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_initial_joint_controller",
            default_value="true",
            description="Whether to spawn and activate the initial joint controller at launch.",
        )
    )
    # launch_rviz 决定是否随控制系统一起打开 RViz。
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="true", description="Launch RViz?"
        )
    )

    # Initialize Arguments
    # LaunchConfiguration 是 launch 系统里的延迟求值参数对象。
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    rws_ip = LaunchConfiguration("rws_ip")
    rws_port = LaunchConfiguration("rws_port")
    configure_via_rws = LaunchConfiguration("configure_via_rws")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    start_initial_joint_controller = LaunchConfiguration("start_initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # robot_description_content 通过 xacro 命令把 Xacro 文件展开成完整 URDF 字符串。
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "rws_ip:=",
            rws_ip,
            " ",
            "rws_port:=",
            rws_port,
            " ",
            "configure_via_rws:=",
            configure_via_rws,
            " ",
        ]
    )
    # robot_description 是 ROS 2 中传递机器人模型的标准参数名。
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # robot_controllers 指向 controller_manager 使用的 YAML 配置。
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # rviz_config_file 指向 MoveIt 配置包里的 RViz 配置。
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "moveit.rviz"]
    )

    # ros2_control_node 就是 controller_manager 的运行实体。
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    # robot_state_publisher 读取 robot_description，并根据 /joint_states 发布 TF。
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # RViz 用于可视化机器人、TF 和 MoveIt 规划结果。
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # 先启动 joint_state_broadcaster。它只负责发布 /joint_states。
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "60",
            "--switch-timeout",
            "60",
        ],
    )

    # joint_state_broadcaster 完成后，再启动初始轨迹控制器。
    # Jazzy 的 spawner 同时加载多个 controller 时可能在 required_state_interfaces 字段上失败，
    # 所以这里坚持用两个 spawner 顺序启动，而不是一个 spawner 带两个 controller 名。
    initial_joint_controller_spawner = Node(
        package="controller_manager",
        condition=IfCondition(start_initial_joint_controller),
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
            "--service-call-timeout",
            "60",
            "--switch-timeout",
            "60",
        ],
    )

    start_initial_joint_controller_after_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[initial_joint_controller_spawner],
        )
    )

    controller_spawners_started = {"value": False}

    def start_controller_spawners_when_services_ready(event):
        text = event.text.decode(errors="ignore")
        if (
            "Resource Manager has been successfully initialized. Starting Controller Manager services"
            not in text
        ):
            return None
        if controller_spawners_started["value"]:
            return None

        controller_spawners_started["value"] = True
        return [
            joint_state_broadcaster_spawner,
        ]

    start_controller_spawners = RegisterEventHandler(
        OnProcessIO(
            target_action=control_node,
            on_stdout=start_controller_spawners_when_services_ready,
            on_stderr=start_controller_spawners_when_services_ready,
        )
    )

    # nodes_to_start 是本 launch 文件最终启动的节点和进程列表。
    nodes_to_start = [
        control_node,
        robot_state_publisher_node,
        rviz_node,
        start_controller_spawners,
        start_initial_joint_controller_after_state_broadcaster,
    ]

    # LaunchDescription 把参数声明和节点启动动作交给 ROS 2 launch 系统执行。
    return LaunchDescription(declared_arguments + nodes_to_start)
