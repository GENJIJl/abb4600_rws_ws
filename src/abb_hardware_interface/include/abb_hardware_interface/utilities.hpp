/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * Modifications Copyright (c) 2022, PickNik Inc
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
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/include/abb_robot_cpp_utilities/initialization.h
// https://github.com/ros-industrial/abb_robot_driver/blob/master/abb_robot_cpp_utilities/include/abb_robot_cpp_utilities/verification.h

#pragma once

#include <string>

#include <abb_egm_rws_managers/rws_manager.h>
#include <string>

namespace abb
{
namespace robot
{
namespace utilities
{
/**
 * \brief 尝试连接机器人控制器的 RWS 服务器。
 *
 * 如果连接建立成功，则返回机器人控制器的结构化描述。
 *
 * \param rws_manager 用于处理与机器人控制器的 RWS 通信。
 * \param robot_controller_id 目标机器人控制器的标识符/昵称。
 * \param no_connection_timeout 表示是否无限期等待机器人控制器连接。
 *
 * \return 机器人控制器的 RobotControllerDescription。
 *
 * \throw std::runtime_error 无法建立连接时抛出。
 */
RobotControllerDescription establishRWSConnection(RWSManager& rws_manager, const std::string& robot_controller_id,
                                                  const bool no_connection_timeout);

/**
 * \brief 验证 RobotWare 版本是否受支持。
 *
 * 注意：目前只支持 [6.07.01, 7.0) 范围内的 RobotWare 版本，即不包含 7.0。
 *
 * \param rw_version 待验证的版本。
 *
 * \throw std::runtime_error RobotWare 版本不受支持时抛出。
 */
void verifyRobotWareVersion(const RobotWareVersion& rw_version);

/**
 * \brief 验证系统中是否存在 RobotWare StateMachine 插件。
 *
 * \param system_indicators 待验证的系统指示信息。
 *
 * \return 如果存在 StateMachine 插件则返回 true。
 */
bool verifyStateMachineAddInPresence(const SystemIndicators& system_indicators);
}  // namespace utilities
}  // namespace robot
}  // namespace abb
