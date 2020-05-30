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

#include <cstdlib>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "utilities.hpp"

namespace free_fleet {

//==============================================================================

double get_yaw_from_quat(const geometry_msgs::Quaternion& _quat)
{
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(_quat, tf2_quat);
  tf2::Matrix3x3 tf2_mat(tf2_quat);
  
  // ignores pitch and roll, but the api call is so nice though
  double yaw;
  double pitch;
  double roll;
  tf2_mat.getEulerYPR(yaw, pitch, roll);
  return yaw;
}

//==============================================================================

std::string generate_random_task_id(size_t length)
{
  auto randchar = []() -> char
  {
      const char charset[] =
      "0123456789"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz";
      const size_t max_index = (sizeof(charset) - 1);
      return charset[ rand() % max_index ];
  };
  std::string str(length,0);
  std::generate_n( str.begin(), length, randchar );
  return str;
}

//==============================================================================

void convert(
    const free_fleet::messages::Location& in,
    ff_rviz_plugins_msgs::Location& out)
{
  out.t = ros::Time(in.sec, in.nanosec);
  out.x = in.x;
  out.y = in.y;
  out.yaw = in.yaw;
  out.level_name = in.level_name;
}

//==============================================================================

void convert(
    const free_fleet::messages::RobotMode& in,
    ff_rviz_plugins_msgs::RobotMode& out)
{
  out.mode = in.mode;
}

//==============================================================================

void convert(
    const free_fleet::messages::RobotState& in,
    ff_rviz_plugins_msgs::RobotState& out)
{
  out.name = in.name;
  out.model = in.model;
  out.task_id = in.task_id;
  convert(in.mode, out.mode);
  out.battery_percent = in.battery_percent;
  convert(in.location, out.location);
  out.path.clear();
  for (const auto& loc : in.path)
  {
    ff_rviz_plugins_msgs::Location loc_msg;
    convert(loc, loc_msg);
    out.path.push_back(loc_msg);
  }
}

//==============================================================================

void convert(
    const std::vector<free_fleet::messages::RobotState>& in, 
    ff_rviz_plugins_msgs::RobotStateArray& out)
{
  out.states.clear();
  for (const auto& rs : in)
  {
    ff_rviz_plugins_msgs::RobotState rs_msg;
    convert(rs, rs_msg);
    out.states.push_back(rs_msg);
  }
}

//==============================================================================

} // namespace free_fleet
