/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef FF_RVIZ_PLUGINS__SRC__UTILITIES_HPP
#define FF_RVIZ_PLUGINS__SRC__UTILITIES_HPP

#include <geometry_msgs/Quaternion.h>

#include <free_fleet/messages/RobotState.hpp>

#include <ff_rviz_plugins_msgs/Location.h>
#include <ff_rviz_plugins_msgs/RobotMode.h>
#include <ff_rviz_plugins_msgs/RobotState.h>
#include <ff_rviz_plugins_msgs/RobotStateArray.h>

namespace free_fleet {

double get_yaw_from_quat(const geometry_msgs::Quaternion& quat);

std::string generate_random_task_id(size_t length);

void convert(
    const free_fleet::messages::Location& in,
    ff_rviz_plugins_msgs::Location& out);

void convert(
    const free_fleet::messages::RobotMode& in,
    ff_rviz_plugins_msgs::RobotMode& out);

void convert(
    const free_fleet::messages::RobotState& in,
    ff_rviz_plugins_msgs::RobotState& out);

void convert(
    const std::vector<free_fleet::messages::RobotState>& in, 
    ff_rviz_plugins_msgs::RobotStateArray& out);

} // namespace free_fleet

#endif // FF_RVIZ_PLUGINS__SRC__UTILITIES_HPP
