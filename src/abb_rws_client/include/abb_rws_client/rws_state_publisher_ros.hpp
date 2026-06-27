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

// 本文件修改自：
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_service_provider/include/abb_rws_service_provider/
// https://github.com/ros-industrial/abb_robot_driver/tree/master/abb_rws_state_publisher/include/abb_rws_state_publisher/

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <abb_egm_rws_managers/rws_manager.h>
#include <abb_egm_rws_managers/system_data_parser.h>

#include <abb_rapid_sm_addin_msgs/msg/runtime_state.hpp>
#include <abb_robot_msgs/msg/rapid_task_state.hpp>
#include <abb_robot_msgs/msg/system_state.hpp>

namespace abb_rws_client
{
class RWSStatePublisherROS
{
public:
  /**
   * \brief 创建状态发布器。
   *
   * \param node ROS 2 节点。
   * \param robot_ip 机器人控制器 RWS 服务器的 IP 地址。
   * \param robot_poty 机器人控制器 RWS 服务器的端口号。
   */
  RWSStatePublisherROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip, unsigned short robot_port);

private:
  /**
   * \brief 定时回调，用于接收并发布机器人系统状态。
   */
  void timer_callback();

  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * \brief 管理与机器人控制器之间的 RWS 通信。
   */
  abb::robot::RWSManager rws_manager_;

  /**
   * \brief 已连接机器人控制器的描述。
   */
  abb::robot::RobotControllerDescription robot_controller_description_;

  /**
   * \brief 关节状态发布器。
   */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  /**
   * \brief 通用系统状态发布器。
   */
  rclcpp::Publisher<abb_robot_msgs::msg::SystemState>::SharedPtr system_state_pub_;

  /**
   * \brief RobotWare StateMachine 插件运行时状态发布器。
   *
   * 注意：仅当系统中存在该插件时使用。
   */
  rclcpp::Publisher<abb_rapid_sm_addin_msgs::msg::RuntimeState>::SharedPtr runtime_state_pub_;

  /**
   * \brief 机器人控制器中定义的各机械单元的运动数据。
   */
  abb::robot::MotionData motion_data_;

  /**
   * \brief 机器人控制器系统状态数据。
   */
  abb::robot::SystemStateData system_state_data_;
};

}  // namespace abb_rws_client
