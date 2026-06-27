/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */
//
// 本文件修改自：
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_service_provider/include/abb_rws_service_provider/
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_state_publisher/include/abb_rws_state_publisher/

#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <abb_egm_rws_managers/rws_manager.h>
#include <abb_egm_rws_managers/system_data_parser.h>

#include <abb_rapid_sm_addin_msgs/msg/runtime_state.hpp>
#include <abb_robot_msgs/msg/system_state.hpp>

#include <abb_rapid_sm_addin_msgs/srv/get_egm_settings.hpp>
#include <abb_rapid_sm_addin_msgs/srv/set_egm_settings.hpp>
#include <abb_rapid_sm_addin_msgs/srv/set_rapid_routine.hpp>
#include <abb_rapid_sm_addin_msgs/srv/set_sg_command.hpp>

#include <abb_robot_msgs/srv/get_file_contents.hpp>
#include <abb_robot_msgs/srv/get_io_signal.hpp>
#include <abb_robot_msgs/srv/get_rapid_bool.hpp>
#include <abb_robot_msgs/srv/get_rapid_dnum.hpp>
#include <abb_robot_msgs/srv/get_rapid_num.hpp>
#include <abb_robot_msgs/srv/get_rapid_string.hpp>
#include <abb_robot_msgs/srv/get_rapid_symbol.hpp>
#include <abb_robot_msgs/srv/get_robot_controller_description.hpp>
#include <abb_robot_msgs/srv/get_speed_ratio.hpp>
#include <abb_robot_msgs/srv/set_file_contents.hpp>
#include <abb_robot_msgs/srv/set_io_signal.hpp>
#include <abb_robot_msgs/srv/set_rapid_bool.hpp>
#include <abb_robot_msgs/srv/set_rapid_dnum.hpp>
#include <abb_robot_msgs/srv/set_rapid_num.hpp>
#include <abb_robot_msgs/srv/set_rapid_string.hpp>
#include <abb_robot_msgs/srv/set_rapid_symbol.hpp>
#include <abb_robot_msgs/srv/set_speed_ratio.hpp>
#include <abb_robot_msgs/srv/trigger_with_result_code.hpp>

namespace abb_rws_client
{
class RWSServiceProviderROS
{
public:
  /**
   * \brief 创建服务服务器。
   *
   * \param node ROS 2 节点。
   * \param robot_ip 机器人控制器 RWS 服务器的 IP 地址。
   * \param robot_poty 机器人控制器 RWS 服务器的端口号。
   */
  RWSServiceProviderROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip, unsigned short robot_port);

  RWSServiceProviderROS() = delete;

  virtual ~RWSServiceProviderROS() = default;

private:
  /**
   * \brief 机器人控制器系统状态消息的回调。
   *
   * \param message 待处理的消息。
   */
  void systemStateCallback(const abb_robot_msgs::msg::SystemState& msg);

  /**
   * \brief RobotWare StateMachine 插件运行时状态消息的回调。
   *
   * 注意：仅当机器人控制器系统中存在该插件时使用。
   *
   * \param message 待处理的消息。
   */
  void runtimeStateCallback(const abb_rapid_sm_addin_msgs::msg::RuntimeState& msg);

  /**
   * \brief 获取文件内容。
   *
   * 该文件必须位于机器人控制器的 home 目录中。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getFileContents(const abb_robot_msgs::srv::GetFileContents::Request::SharedPtr req,
                       abb_robot_msgs::srv::GetFileContents::Response::SharedPtr res);
  /**
   * \brief 获取 IO 信号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getIOSignal(const abb_robot_msgs::srv::GetIOSignal::Request::SharedPtr req,
                   abb_robot_msgs::srv::GetIOSignal::Response::SharedPtr res);

  /**
   * \brief 获取 RAPID `bool` 符号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getRAPIDBool(const abb_robot_msgs::srv::GetRAPIDBool::Request::SharedPtr req,
                    abb_robot_msgs::srv::GetRAPIDBool::Response::SharedPtr res);

  /**
   * \brief 获取 RAPID `dnum` 符号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getRAPIDDNum(const abb_robot_msgs::srv::GetRAPIDDnum::Request::SharedPtr req,
                    abb_robot_msgs::srv::GetRAPIDDnum::Response::SharedPtr res);

  /**
   * \brief 获取 RAPID `num` 符号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getRAPIDNum(const abb_robot_msgs::srv::GetRAPIDNum::Request::SharedPtr req,
                   abb_robot_msgs::srv::GetRAPIDNum::Response::SharedPtr res);

  /**
   * \brief 获取 RAPID `string` 符号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getRAPIDString(const abb_robot_msgs::srv::GetRAPIDString::Request::SharedPtr req,
                      abb_robot_msgs::srv::GetRAPIDString::Response::SharedPtr res);

  /**
   * \brief 获取 RAPID 符号（原始文本格式）。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getRAPIDSymbol(const abb_robot_msgs::srv::GetRAPIDSymbol::Request::SharedPtr req,
                      abb_robot_msgs::srv::GetRAPIDSymbol::Response::SharedPtr res);

  /**
   * \brief 获取已连接机器人控制器的描述。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getRCDescription(const abb_robot_msgs::srv::GetRobotControllerDescription::Request::SharedPtr req,
                        abb_robot_msgs::srv::GetRobotControllerDescription::Response::SharedPtr res);

  /**
   * \brief 获取 RAPID 运动的控制器速度比例（范围 [0, 100]）。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getSpeedRatio(const abb_robot_msgs::srv::GetSpeedRatio::Request::SharedPtr req,
                     abb_robot_msgs::srv::GetSpeedRatio::Response::SharedPtr res);

  /**
   * \brief 将所有 RAPID 程序指针设置到各自的 main 过程。
   * 启动所有 RAPID 程序。
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool ppToMain(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 发出信号，表示所有 RAPID 程序应运行自定义 RAPID 例程。
   *
   * 注意：
   * - 需要 StateMachine 插件。
   * - 需要提前指定目标 RAPID 例程（每个 RAPID 任务一个）。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool runRAPIDRoutine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                       abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 发出信号，表示所有 RAPID 程序应运行 SmartGripper 命令。
   *
   * 注意：
   * - 需要 StateMachine 插件。
   * - 需要提前指定目标 SmartGripper 命令（每个 RAPID 任务一个）。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool runSGRoutine(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                    abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 设置文件内容。
   *
   * 该文件会上传到机器人控制器的 home 目录。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setFileContents(const abb_robot_msgs::srv::SetFileContents::Request::SharedPtr req,
                       abb_robot_msgs::srv::SetFileContents::Response::SharedPtr res);

  /**
   * \brief 设置 IO 信号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setIOSignal(const abb_robot_msgs::srv::SetIOSignal::Request::SharedPtr req,
                   abb_robot_msgs::srv::SetIOSignal::Response::SharedPtr res);

  /**
   * \brief 关闭电机。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setMotorsOff(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                    abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 打开电机。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setMotorsOn(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                   abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 设置 RAPID `bool` 符号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setRAPIDBool(const abb_robot_msgs::srv::SetRAPIDBool::Request::SharedPtr req,
                    abb_robot_msgs::srv::SetRAPIDBool::Response::SharedPtr res);

  /**
   * \brief 设置 RAPID `dnum` 符号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setRAPIDDNum(const abb_robot_msgs::srv::SetRAPIDDnum::Request::SharedPtr req,
                    abb_robot_msgs::srv::SetRAPIDDnum::Response::SharedPtr res);

  /**
   * \brief 设置 RAPID `num` 符号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setRAPIDNum(const abb_robot_msgs::srv::SetRAPIDNum::Request::SharedPtr req,
                   abb_robot_msgs::srv::SetRAPIDNum::Response::SharedPtr res);

  /**
   * \brief 设置 RAPID `string` 符号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setRAPIDString(const abb_robot_msgs::srv::SetRAPIDString::Request::SharedPtr req,
                      abb_robot_msgs::srv::SetRAPIDString::Response::SharedPtr res);

  /**
   * \brief 设置 RAPID 符号。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setRAPIDSymbol(const abb_robot_msgs::srv::SetRAPIDSymbol::Request::SharedPtr req,
                      abb_robot_msgs::srv::SetRAPIDSymbol::Response::SharedPtr res);

  /**
   * \brief 设置 RAPID 运动的控制器速度比例（范围 [0, 100]）。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setSpeedRatio(const abb_robot_msgs::srv::SetSpeedRatio::Request::SharedPtr req,
                     abb_robot_msgs::srv::SetSpeedRatio::Response::SharedPtr res);

  /**
   * \brief 启动所有 RAPID 程序。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool startRAPID(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                  abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 停止所有 RAPID 程序。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool stopRAPID(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                 abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 获取特定 RAPID 任务使用的 EGM 设置。
   *
   * 注意：需要 StateMachine 插件。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool getEGMSettings(const abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Request::SharedPtr req,
                      abb_rapid_sm_addin_msgs::srv::GetEGMSettings::Response::SharedPtr res);

  /**
   * \brief 设置特定 RAPID 任务使用的 EGM 设置。
   *
   * 注意：需要 StateMachine 插件。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setEGMSettings(const abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Request::SharedPtr req,
                      abb_rapid_sm_addin_msgs::srv::SetEGMSettings::Response::SharedPtr res);

  /**
   * \brief 为特定 RAPID 任务设置目标自定义 RAPID 例程。
   *
   * 注意：需要 StateMachine 插件。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setRAPIDRoutine(const abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Request::SharedPtr req,
                       abb_rapid_sm_addin_msgs::srv::SetRAPIDRoutine::Response::SharedPtr res);

  /**
   * \brief 为特定 RAPID 任务设置目标 SmartGripper 命令。
   *
   * 注意：需要 StateMachine 插件。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool setSGCommand(const abb_rapid_sm_addin_msgs::srv::SetSGCommand::Request::SharedPtr req,
                    abb_rapid_sm_addin_msgs::srv::SetSGCommand::Response::SharedPtr res);

  /**
   * \brief 为所有 RAPID 程序启动 EGM 关节运动。
   *
   * 注意：需要 StateMachine 插件。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool startEGMJoint(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                     abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 为所有 RAPID 程序启动 EGM 姿态运动。
   *
   * 注意：需要 StateMachine 插件。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool startEGMPose(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                    abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 为所有 RAPID 程序启动 EGM 位置流。
   *
   * 注意：需要 StateMachine 插件。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool startEGMStream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                      abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 停止所有 RAPID 程序的 EGM 运动。
   *
   * 注意：需要 StateMachine 插件。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool stopEGM(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
               abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 停止所有 RAPID 程序的 EGM 位置流。
   *
   * 注意：需要 StateMachine 插件。
   *
   * \param req 待处理的请求。
   * \param res 用于保存结果的响应。
   *
   * \return 如果请求已处理则返回 true。
   */
  bool stopEGMStream(const abb_robot_msgs::srv::TriggerWithResultCode::Request::SharedPtr req,
                     abb_robot_msgs::srv::TriggerWithResultCode::Response::SharedPtr res);

  /**
   * \brief 验证自动模式是否处于活动状态。
   *
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果自动模式处于活动状态则返回 true。
   */
  bool verifyAutoMode(uint16_t& result_code, std::string& message);

  /**
   * \brief 验证文件名非空。
   *
   * \param filename 待验证的文件名。
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果文件名非空则返回 true。
   */
  bool verifyArgumentFilename(const std::string& filename, uint16_t& result_code, std::string& message);

  /**
   * \brief 验证 RAPID 符号路径不包含空子组件。
   *
   * \param path 待验证的路径。
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果 RAPID 符号路径不包含空子组件则返回 true。
   */
  bool verifyArgumentRAPIDSymbolPath(const abb_robot_msgs::msg::RAPIDSymbolPath& path, uint16_t& result_code,
                                     std::string& message);

  /**
   * \brief 验证 RAPID 任务名非空。
   *
   * \param task 待验证的任务名。
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果 RAPID 任务名非空则返回 true。
   */
  bool verifyArgumentRAPIDTask(const std::string& task, uint16_t& result_code, std::string& message);

  /**
   * \brief 验证 IO 信号名非空。
   *
   * \param signal 待验证的信号名。
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果 IO 信号名非空则返回 true。
   */
  bool verifyArgumentSignal(const std::string& signal, uint16_t& result_code, std::string& message);

  /**
   * \brief 验证电机已关闭。
   *
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果电机已关闭则返回 true。
   */
  bool verifyMotorsOff(uint16_t& result_code, std::string& message);

  /**
   * \brief 验证电机已打开。
   *
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果电机已打开则返回 true。
   */
  bool verifyMotorsOn(uint16_t& result_code, std::string& message);

  /**
   * \brief 验证是否已接收到 StateMachine 插件实例的运行时状态。
   *
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果已接收到运行时状态则返回 true。
   */
  bool verifySMAddinRuntimeStates(uint16_t& result_code, std::string& message);

  /**
   * \brief 验证 RAPID 任务是否使用 StateMachine 插件实例。
   *
   * \param task 待检查的任务名。
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果 RAPID 任务使用 StateMachine 插件实例则返回 true。
   */
  bool verifySMAddinTaskExist(const std::string& task, uint16_t& result_code, std::string& message);

  /**
   * \brief 验证 RAPID 任务是否使用 StateMachine 插件实例且该实例已初始化。
   *
   * \param task 待检查的任务名。
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果 RAPID 任务使用 StateMachine 插件实例且该实例已初始化则返回 true。
   */
  bool verifySMAddinTaskInitialized(const std::string& task, uint16_t& result_code, std::string& message);

  /**
   * \brief 验证 RAPID 是否正在运行。
   *
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果 RAPID 正在运行则返回 true。
   */
  bool verifyRAPIDRunning(uint16_t& result_code, std::string& message);

  /**
   * \brief 验证 RAPID 是否已停止。
   *
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果 RAPID 已停止则返回 true。
   */
  bool verifyRAPIDStopped(uint16_t& result_code, std::string& message);

  /**
   * \brief 验证 RWS 通信管理器是否就绪。
   *
   * \param[out] result_code 数值错误码。
   * \param[out] message 用于保存可能错误消息的容器。
   *
   * \return 如果 RWS 通信管理器就绪则返回 true。
   */
  bool verifyRWSManagerReady(uint16_t& result_code, std::string& message);

  rclcpp::Node::SharedPtr node_;

  /**
   * \brief 管理与机器人控制器之间的 RWS 通信。
   */
  abb::robot::RWSManager rws_manager_;

  /**
   * \brief 已连接机器人控制器的描述。
   */
  abb::robot::RobotControllerDescription robot_controller_description_;

  /**
   * \brief 机器人控制器系统状态订阅器。
   */
  rclcpp::Subscription<abb_robot_msgs::msg::SystemState>::SharedPtr system_state_sub_;

  /**
   * \brief RobotWare StateMachine 插件运行时状态订阅器。
   */
  rclcpp::Subscription<abb_rapid_sm_addin_msgs::msg::RuntimeState>::SharedPtr runtime_state_sub_;

  /**
   * \brief 已提供的 ROS 核心服务列表。
   */
  std::vector<rclcpp::ServiceBase::SharedPtr> core_services_;

  /**
   * \brief 已提供的 ROS 状态机服务列表。
   */
  std::vector<rclcpp::ServiceBase::SharedPtr> sm_services_;

  /**
   * \brief 最近一次已知的机器人控制器系统状态。
   */
  abb_robot_msgs::msg::SystemState system_state_;

  /**
   * \brief 最近一次已知的 RobotWare StateMachine 插件运行时状态。
   */
  abb_rapid_sm_addin_msgs::msg::RuntimeState runtime_state_;
};

}  // namespace abb_rws_client
