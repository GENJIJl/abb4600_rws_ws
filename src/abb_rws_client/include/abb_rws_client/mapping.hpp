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
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/include/abb_robot_cpp_utilities/mapping.h

#pragma once

#include <string>
#include <vector>

#include <abb_egm_rws_managers/utilities.h>
#include <abb_librws/rws_rapid.h>

#include <abb_rapid_msgs/msg/tool_data.hpp>
#include <abb_rapid_msgs/msg/w_obj_data.hpp>
#include <abb_rapid_sm_addin_msgs/msg/egm_settings.hpp>

namespace abb
{
namespace robot
{
namespace utilities
{
/**
 * \brief 将 RAPID 任务执行状态映射为 ROS 表示。
 *
 * \param state 待映射的状态。
 *
 * \return 包含映射后状态的 uint8_t。
 */
uint8_t map(const rws::RWSInterface::RAPIDTaskExecutionState state);

/**
 * \brief 将 RobotWare StateMachine 插件状态映射为 ROS 表示。
 *
 * \param state 待映射的状态。
 *
 * \return 包含映射后状态的 uint8_t。
 */
uint8_t mapStateMachineState(const rws::RAPIDNum& state);

/**
 * \brief 将 RobotWare StateMachine 插件的 EGM 动作映射为 ROS 表示。
 *
 * \param action 待映射的动作。
 *
 * \return 包含映射后状态的 uint8_t。
 */
uint8_t mapStateMachineEGMAction(const rws::RAPIDNum& action);

/**
 * \brief 将 RobotWare StateMachine 插件的 SmartGripper 命令映射为 RWS 表示。
 *
 * \param command 待映射的命令。
 *
 * \return 包含映射后命令的 unsigned int。
 *
 * \throw std::runtime_error 命令未知时抛出。
 */
unsigned int mapStateMachineSGCommand(const unsigned int command);

/**
 * \brief 将 RAPID `pos` 数据类型从 RWS 表示映射为 ROS 表示。
 *
 * \param rws_pos 待映射的 RWS pos。
 *
 * \return 包含映射后数据的 abb_rapid_msgs::Pos。
 */
abb_rapid_msgs::msg::Pos map(const rws::Pos& rws_pos);

/**
 * \brief 将 RAPID `orient` 数据类型从 RWS 表示映射为 ROS 表示。
 *
 * \param rws_orient 待映射的 RWS orient。
 *
 * \return 包含映射后数据的 abb_rapid_msgs::Orient。
 */
abb_rapid_msgs::msg::Orient map(const rws::Orient& rws_orient);

/**
 * \brief 将 RAPID `pose` 数据类型从 RWS 表示映射为 ROS 表示。
 *
 * \param rws_pose 待映射的 RWS pose。
 *
 * \return 包含映射后数据的 abb_rapid_msgs::Pose。
 */
abb_rapid_msgs::msg::Pose map(const rws::Pose& rws_pose);

/**
 * \brief 将 RAPID `loaddata` 数据类型从 RWS 表示映射为 ROS 表示。
 *
 * \param rws_loaddata 待映射的 RWS loaddata。
 *
 * \return 包含映射后数据的 abb_rapid_msgs::LoadData。
 */
abb_rapid_msgs::msg::LoadData map(const rws::LoadData& rws_loaddata);

/**
 * \brief 将 RAPID `tooldata` 数据类型从 RWS 表示映射为 ROS 表示。
 *
 * \param rws_tooldata 待映射的 RWS tooldata。
 *
 * \return 包含映射后数据的 abb_rapid_msgs::ToolData。
 */
abb_rapid_msgs::msg::ToolData map(const rws::ToolData& rws_tooldata);

/**
 * \brief 将 RAPID `wobjdata` 数据类型从 RWS 表示映射为 ROS 表示。
 *
 * \param rws_wobjdata 待映射的 RWS wobjdata。
 *
 * \return 包含映射后数据的 abb_rapid_msgs::WObjData。
 */
abb_rapid_msgs::msg::WObjData map(const rws::WObjData& rws_wobjdata);

/**
 * \brief 将 RobotWare StateMachine 插件中的 RAPID `EGMSettings` 数据类型从 RWS 表示映射为 ROS 表示。
 *
 * \param rws_egm_settings 待映射的 RWS EGMSettings。
 *
 * \return 包含映射后数据的 abb_rapid_sm_addin_msgs::EGMSettings。
 */
abb_rapid_sm_addin_msgs::msg::EGMSettings map(const rws::RWSStateMachineInterface::EGMSettings& rws_egm_settings);

/**
 * \brief 将 RAPID `pos` 数据类型从 ROS 表示映射为 RWS 表示。
 *
 * \param ros_pos 待映射的 ROS pos。
 *
 * \return 包含映射后数据的 rws::Pos。
 */
rws::Pos map(const abb_rapid_msgs::msg::Pos& ros_pos);

/**
 * \brief 将 RAPID `orient` 数据类型从 ROS 表示映射为 RWS 表示。
 *
 * \param ros_orient 待映射的 ROS orient。
 *
 * \return 包含映射后数据的 rws::Orient。
 */
rws::Orient map(const abb_rapid_msgs::msg::Orient& ros_orient);

/**
 * \brief 将 RAPID `pose` 数据类型从 ROS 表示映射为 RWS 表示。
 *
 * \param ros_pose 待映射的 ROS pose。
 *
 * \return 包含映射后数据的 rws::Pose。
 */
rws::Pose map(const abb_rapid_msgs::msg::Pose& ros_pose);

/**
 * \brief 将 RAPID `loaddata` 数据类型从 ROS 表示映射为 RWS 表示。
 *
 * \param ros_loaddata 待映射的 ROS loaddata。
 *
 * \return 包含映射后数据的 rws::LoadData。
 */
rws::LoadData map(const abb_rapid_msgs::msg::LoadData& ros_loaddata);

/**
 * \brief 将 RAPID `tooldata` 数据类型从 ROS 表示映射为 RWS 表示。
 *
 * \param ros_tooldata 待映射的 ROS tooldata。
 *
 * \return 包含映射后数据的 rws::ToolData。
 */
rws::ToolData map(const abb_rapid_msgs::msg::ToolData& ros_tooldata);

/**
 * \brief 将 RAPID `wobjdata` 数据类型从 ROS 表示映射为 RWS 表示。
 *
 * \param ros_wobjdata 待映射的 ROS wobjdata。
 *
 * \return 包含映射后数据的 rws::WObjData。
 */
rws::WObjData map(const abb_rapid_msgs::msg::WObjData& ros_wobjdata);

/**
 * \brief 将 RobotWare StateMachine 插件中的 RAPID `EGMSettings` 数据类型从 ROS 表示映射为 RWS 表示。
 *
 * \param ros_egm_settings 待映射的 ROS EGMSettings。
 *
 * \return 包含映射后数据的 rws::RWSStateMachineInterface::EGMSettings。
 */
rws::RWSStateMachineInterface::EGMSettings map(const abb_rapid_sm_addin_msgs::msg::EGMSettings& ros_egm_settings);

/**
 * \brief 将 EGM 状态映射为 ROS 表示。
 *
 * \param state 待映射的状态。
 *
 * \return 包含映射后状态的 uint8_t。
 */
uint8_t map(const egm::wrapper::Status::EGMState state);

/**
 * \brief 将电机状态映射为 ROS 表示。
 *
 * \param state 待映射的状态。
 *
 * \return 包含映射后状态的 uint8_t。
 */
uint8_t map(const egm::wrapper::Status::MotorState state);

/**
 * \brief 将 RAPID 执行状态映射为 ROS 表示。
 *
 * \param state 待映射的状态。
 *
 * \return 包含映射后状态的 uint8_t。
 */
uint8_t map(const egm::wrapper::Status::RAPIDExecutionState rapid_execution_state);

/**
 * \brief 将 vector 映射为字符串，例如用于日志。
 *
 * \param vector 待映射的 vector。
 *
 * \return 映射后的 std::string。
 *
 * \throw std::runtime 映射失败时抛出。
 */
template <typename type>
std::string mapVectorToString(const std::vector<type>& vector);

}  // namespace utilities
}  // namespace robot
}  // namespace abb
