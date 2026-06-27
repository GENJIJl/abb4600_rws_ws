// Copyright 2020 ROS2-Control Development Team
// Modifications Copyright 2022 PickNik Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <abb_hardware_interface/abb_hardware_interface.hpp>
#include <abb_hardware_interface/utilities.hpp>

#include <sstream>

using namespace std::chrono_literals;

namespace abb_hardware_interface
{
// 激活阶段最多等待 EGM 报文的次数。
// 每次等待 500 ms，再额外 sleep 500 ms，因此最坏连接等待时间约为 100 s。
static constexpr size_t NUM_CONNECTION_TRIES = 100;

// 当前硬件插件统一使用的 ROS 日志器名称，便于在 ros2 日志中过滤 ABB 硬件接口输出。
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ABBSystemHardware");

CallbackReturn ABBSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  // on_init 在 controller_manager 加载硬件插件时调用，是整个硬件接口的初始化入口。
  // info 来自 robot_description 中的 ros2_control 标签，包含关节、接口和 hardware_parameters。
  // 先调用父类实现，让 ros2_control 保存并解析基础 HardwareInfo 到 info_。
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // 校验 ros2_control xacro 中配置的接口。
  // 这个驱动只导出 position 和 velocity 两类接口，并且后续代码按固定下标读取：
  // command_interfaces[0] -> position，command_interfaces[1] -> velocity；
  // state_interfaces[0]   -> position，state_interfaces[1]   -> velocity。
  // 因此这里必须提前检查数量和顺序，避免运行时绑定到错误的数据字段。
  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                   joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  // 构造 robot_controller_description_，这是 abb_libegm/abb_librws 使用的控制器描述对象。
  // 默认通过 RWS 从真实控制器或 RobotStudio 获取完整描述。
  // 如果 configure_via_rws 设置为 false/False，则根据 ros2_control xacro 中的关节信息本地构造。
  // 未配置 configure_via_rws 时按 true 处理，保持“优先相信控制器实际配置”的默认行为。
  const auto configure_it = info_.hardware_parameters.find("configure_via_rws");
  const bool configure_via_rws = configure_it == info_.hardware_parameters.end()                    ? true :
                                 configure_it->second == "false" || configure_it->second == "False" ? false :
                                                                                                      true;

  if (configure_via_rws)
  {
    // 真实控制器或 RobotStudio 模式下，优先通过 RWS 读取 ABB 控制器信息。
    // rws_ip/rws_port 来自 ros2_control 的 hardware_parameters。
    RCLCPP_INFO_STREAM(LOGGER, "Generating robot controller description from RWS.");
    const auto rws_port = stoi(info_.hardware_parameters["rws_port"]);
    const auto rws_ip = info_.hardware_parameters["rws_ip"];

    // xacro 中用 "None" 表示未设置 IP；这里不能继续连接，否则 RWSManager 会拿到无效地址。
    if (rws_ip == "None")
    {
      RCLCPP_FATAL(LOGGER, "RWS IP not specified");
      return CallbackReturn::ERROR;
    }

    // 用默认 RWS 账号建立会话，并从控制器读取机器人控制器描述。
    // 不给标准关节名添加机器人型号前缀，保持 joint_1..joint_6 与 URDF/controller 配置一致。
    abb::robot::RWSManager rws_manager(rws_ip, rws_port, "Default User", "robotics");
    robot_controller_description_ = abb::robot::utilities::establishRWSConnection(rws_manager, "", true);
  }
  else
  {
    // 不通过 RWS 时，根据 ros2_control 中列出的 joints 构造最小可用控制器描述。
    // 这种模式适合离线测试、没有 RWS 可访问但仍需要初始化 EGM 数据结构的场景。
    RCLCPP_INFO_STREAM(LOGGER, "Generating robot controller description from HardwareInfo.");

    // 添加头部信息。EGM 管理器会依据 RobotWare 版本选择对应的数据解释方式。
    auto header{ robot_controller_description_.mutable_header() };
    // OmniCore 控制器的 RobotWare 版本 >= 7.0.0。
    header->mutable_robot_ware_version()->set_major_number(7);
    header->mutable_robot_ware_version()->set_minor_number(3);
    header->mutable_robot_ware_version()->set_patch_number(2);

    // 添加系统指示信息，声明当前控制器描述支持 EGM 选项。
    auto system_indicators{ robot_controller_description_.mutable_system_indicators() };
    system_indicators->mutable_options()->set_egm(true);

    // 添加单个机械单元组。组名为空字符串时，后续读取的硬件参数键名就是 "egm_port"。
    auto mug{ robot_controller_description_.add_mechanical_units_groups() };
    mug->set_name("");

    // 向机械单元组添加单个 TCP 机器人，并把 ros2_control 中的关节数量作为轴数。
    auto robot{ mug->mutable_robot() };
    robot->set_type(abb::robot::MechanicalUnit_Type_TCP_ROBOT);
    robot->set_axes_total(info_.joints.size());
    robot->set_mode(abb::robot::MechanicalUnit_Mode_ACTIVATED);

    // 向机器人添加标准化关节描述。每个关节至少需要名称、关节类型和位置范围。
    for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      const hardware_interface::ComponentInfo& joint = info_.joints[i];
      // 除非明确指定，否则默认关节类型为 revolute。
      // 按 sdformat 约定检查 joint.parameters 中是否存在非 revolute 的 "type" 键：
      // http://sdformat.org/spec?elem=joint。
      const auto type_it = joint.parameters.find("type");
      const bool is_revolute = type_it != joint.parameters.end() && type_it->second != "revolute" ? false : true;

      // 从 position 命令接口中读取关节范围。
      // ros2_control 的 InterfaceInfo::min/max 是字符串，这里转换成 double 后写入 ABB 描述。
      for (const hardware_interface::InterfaceInfo& joint_info : joint.command_interfaces)
      {
        if (joint_info.name == hardware_interface::HW_IF_POSITION)
        {
          const double min = std::stod(joint_info.min);
          const double max = std::stod(joint_info.max);

          abb::robot::StandardizedJoint* p_joint = robot->add_standardized_joints();
          // standardized_name 必须与后续导出的接口名可对应，
          // 否则 controller 无法按关节名匹配数据。
          p_joint->set_standardized_name(joint.name);
          p_joint->set_rotating_move(is_revolute);
          p_joint->set_lower_joint_bound(min);
          p_joint->set_upper_joint_bound(max);

          RCLCPP_INFO(LOGGER, "Configured component %s of type %s with range [%.3f, %.3f]", joint.name.c_str(),
                      joint.type.c_str(), min, max);
          break;
        }
      }
    }
  }

  RCLCPP_INFO_STREAM(LOGGER, "Robot controller description:\n"
                                 << abb::robot::summaryText(robot_controller_description_));

  // 配置 EGM。EGM 负责通过 UDP 与 ABB 控制器交换高速关节状态和命令。
  RCLCPP_INFO(LOGGER, "Configuring EGM interface...");
  const auto initial_hold_it = info_.hardware_parameters.find("initial_hold_duration_sec");
  if (initial_hold_it != info_.hardware_parameters.end())
  {
    initial_hold_duration_sec_ = std::stod(initial_hold_it->second);
  }
  RCLCPP_INFO(LOGGER, "Initial EGM hold duration: %.3f sec", initial_hold_duration_sec_);

  // 根据机器人控制器描述初始化 motion_data_。
  // motion_data_ 是本插件的核心数据缓存：
  // state 字段由 read() 更新，command 字段由 write() 发送。
  try
  {
    abb::robot::initializeMotionData(motion_data_, robot_controller_description_);
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize motion data from robot controller description");
    return CallbackReturn::ERROR;
  }

  // 为每个机械单元组创建通道配置。
  // 每个 mechanical unit group 都需要一个 EGM UDP 通道。
  // 参数键名采用 "<group.name()>egm_port"；组名为空时就是 "egm_port"。
  std::vector<abb::robot::EGMManager::ChannelConfiguration> channel_configurations;
  for (const auto& group : robot_controller_description_.mechanical_units_groups())
  {
    try
    {
      // EGMManager 需要 uint16_t 端口号，因此先从 hardware_parameters 字符串转为整数再收窄。
      const auto egm_port = stoi(info_.hardware_parameters[group.name() + "egm_port"]);
      const auto channel_configuration =
          abb::robot::EGMManager::ChannelConfiguration{ static_cast<uint16_t>(egm_port), group };
      channel_configurations.emplace_back(channel_configuration);
      RCLCPP_INFO_STREAM(LOGGER,
                         "Configuring EGM for mechanical unit group " << group.name() << " on port " << egm_port);
    }
    catch (std::invalid_argument& e)
    {
      RCLCPP_FATAL_STREAM(LOGGER, "EGM port for mechanical unit group \"" << group.name()
                                                                          << "\" not specified in hardware parameters");
      return CallbackReturn::ERROR;
    }
  }

  // 创建 EGM 管理器后，UDP socket 已按通道配置准备好；
  // 真正的数据等待在 on_activate() 中完成。
  try
  {
    egm_manager_ = std::make_unique<abb::robot::EGMManager>(channel_configurations);
  }
  catch (std::runtime_error& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize EGM connection");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ABBSystemHardware::export_state_interfaces()
{
  // export_state_interfaces 把 ABB 侧读取到的关节状态暴露给 ros2_control。
  // StateInterface 持有 motion_data_ 中 state 字段的指针，所以后续 read() 更新 motion_data_ 后，
  // controller_manager 读取到的状态会自动变为最新值。
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        // TODO(seng): 考虑修改机器人描述中的关节名，使其与 ABB 机器人描述中的名称一致，
        // 从而避免在这里剥离前缀。
        // ABB 描述中的名称可能带有机械单元前缀，这里截取从 "joint" 开始的部分，
        // 使接口名与 ros2_control/URDF 中常见的 joint_1、joint_2 等名称一致。
        const auto pos = joint.name.find("joint");
        const auto joint_name = joint.name.substr(pos);
        // 对同一个关节分别导出位置和速度状态接口，控制器可按需声明并读取。
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_POSITION, &joint.state.position));
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity));
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ABBSystemHardware::export_command_interfaces()
{
  // export_command_interfaces 把 ros2_control 的命令接口绑定到 ABB motion_data_。
  // CommandInterface 持有 motion_data_ 中 command 字段的指针，控制器写入命令后，
  // write() 会把这些命令通过 EGMManager 发送给 ABB 控制器。
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        // TODO(seng): 考虑修改机器人描述中的关节名，使其与 ABB 机器人描述中的名称一致，
        // 从而避免在这里剥离前缀。
        // 与状态接口保持相同的关节名规则，保证 controller 同时能匹配 state 和 command。
        const auto pos = joint.name.find("joint");
        const auto joint_name = joint.name.substr(pos);
        // 对同一个关节分别导出位置和速度命令接口。
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_POSITION, &joint.command.position));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint.command.velocity));
      }
    }
  }

  return command_interfaces;
}

CallbackReturn ABBSystemHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  // on_activate 在硬件接口激活时调用。
  // 此时 controller_manager 已完成接口导出，硬件开始进入可读写状态；
  // 这里等待 ABB 控制器发来首个 EGM 报文。
  size_t counter = 0;
  RCLCPP_INFO(LOGGER, "Connecting to robot...");
  while (rclcpp::ok() && counter++ < NUM_CONNECTION_TRIES)
  {
    // 等待任一已配置 EGM 通道上的消息。
    // 收到消息说明控制器端 EGM RAPID 程序已开始发送 UDP 数据。
    if (egm_manager_->waitForMessage(500))
    {
      RCLCPP_INFO(LOGGER, "Connected to robot");
      break;
    }

    RCLCPP_INFO(LOGGER, "Not connected to robot...");
    if (counter == NUM_CONNECTION_TRIES)
    {
      RCLCPP_ERROR(LOGGER, "Failed to connect to robot");
      return CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(500ms);
  }

  // 激活后先读一次当前机器人状态，再把命令位置初始化为当前位置。
  // 这样控制器刚启动时不会因为 command 默认值为 0 而产生突跳命令。
  egm_manager_->read(motion_data_);
  holdCurrentPosition();
  egm_manager_->write(motion_data_);
  activation_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();

  RCLCPP_INFO(LOGGER, "ros2_control hardware interface was successfully started!");

  return CallbackReturn::SUCCESS;
}

return_type ABBSystemHardware::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // read 在 controller_manager 更新循环中调用，用于从 EGM 读取机器人状态。
  // time/period 由 ros2_control 传入，本实现不需要按时间补偿，只直接刷新 motion_data_。
  egm_manager_->read(motion_data_);
  return return_type::OK;
}

return_type ABBSystemHardware::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  // write 在 controller_manager 更新循环中调用，用于把控制命令通过 EGM 写给机器人。
  // 控制器写入的 command.position/command.velocity 已经通过 CommandInterface 指针落在 motion_data_ 中。
  if (initial_hold_duration_sec_ > 0.0)
  {
    const auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    if ((now - activation_time_).seconds() < initial_hold_duration_sec_)
    {
      holdCurrentPosition();
    }
  }

  static std::size_t write_log_counter = 0;
  if (++write_log_counter % 250 == 0)
  {
    for (const auto& group : motion_data_.groups)
    {
      for (const auto& unit : group.units)
      {
        if (unit.active && unit.type == abb::robot::MechanicalUnit_Type_TCP_ROBOT && !unit.joints.empty())
        {
          const auto& joint = unit.joints.front();
          const auto& status = group.egm_channel_data.input.status();
          static constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
          RCLCPP_INFO(
              LOGGER,
              "EGM write debug joint_1: active=%d, rapid=%d, egm=%d, motor=%d, convergence=%d, utilization=%.3f, state=%.3f deg, command=%.3f deg, velocity=%.3f deg/s",
              group.egm_channel_data.is_active ? 1 : 0,
              status.rapid_execution_state(),
              status.egm_state(),
              status.motor_state(),
              status.has_egm_convergence_met() && status.egm_convergence_met() ? 1 : 0,
              status.has_utilization_rate() ? status.utilization_rate() : -1.0,
              joint.state.position * RAD_TO_DEG,
              joint.command.position * RAD_TO_DEG,
              joint.command.velocity * RAD_TO_DEG);
          std::ostringstream order_stream;
          order_stream << "EGM joint order:";
          for (const auto& ordered_joint : unit.joints)
          {
            order_stream << " " << ordered_joint.name
                         << "(state=" << ordered_joint.state.position * RAD_TO_DEG
                         << ", cmd=" << ordered_joint.command.position * RAD_TO_DEG << ")";
          }
          RCLCPP_INFO(LOGGER, "%s", order_stream.str().c_str());
          return_type result = return_type::OK;
          egm_manager_->write(motion_data_);
          return result;
        }
      }
    }
  }
  egm_manager_->write(motion_data_);
  return return_type::OK;
}

void ABBSystemHardware::holdCurrentPosition()
{
  for (auto& group : motion_data_.groups)
  {
    for (auto& unit : group.units)
    {
      for (auto& joint : unit.joints)
      {
        joint.command.position = joint.state.position;
        joint.command.velocity = 0.0;
      }
    }
  }
}

}  // namespace abb_hardware_interface

#include "pluginlib/class_list_macros.hpp"

// 将 ABBSystemHardware 注册为 pluginlib 插件，
// 使 controller_manager 能按 XML 插件描述动态加载该类。
PLUGINLIB_EXPORT_CLASS(abb_hardware_interface::ABBSystemHardware, hardware_interface::SystemInterface)
