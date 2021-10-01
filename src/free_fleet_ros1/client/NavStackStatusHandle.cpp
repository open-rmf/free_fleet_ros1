/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <free_fleet/Console.hpp>
#include <free_fleet_ros1/client/NavStackStatusHandle.hpp>

#include "internal_NavStackStatusHandle.hpp"

namespace free_fleet_ros1 {
namespace client {

//==============================================================================
auto NavStackStatusHandle::Implementation::make(
  const std::string& map_frame,
  const std::string& robot_frame,
  int update_frequency,
  std::shared_ptr<NavStackData> nav_stack_data)
-> std::shared_ptr<NavStackStatusHandle>
{
  
  return nullptr;
}

//==============================================================================
void NavStackStatusHandle::Implementation::update_thread_fn()
{
}

//==============================================================================
void NavStackStatusHandle::Implementation::start()
{
  ffinfo << "Starting NavStackStatusHandle thread.\n";
  thread = std::thread(std::bind(&Implementation::update_thread_fn, this));
}

//==============================================================================
NavStackStatusHandle::NavStackStatusHandle()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
rmf_traffic::Time NavStackStatusHandle::time() const
{
  ros::Time now = ros::Time::now();
  auto output =
    rmf_traffic::Time() +
    std::chrono::seconds(now.sec) +
    std::chrono::nanoseconds(now.nsec);                                    
  return output;
}

//==============================================================================
free_fleet::messages::Location NavStackStatusHandle::location() const
{
  std::lock_guard<std::mutex> lock(_pimpl->nav_stack_data->mutex);
  return _pimpl->nav_stack_data->location.value();
}

//==============================================================================
free_fleet::messages::RobotMode NavStackStatusHandle::mode() const
{
  std::lock_guard<std::mutex> lock(_pimpl->nav_stack_data->mutex);
  return _pimpl->nav_stack_data->mode.value();
}

//==============================================================================
double NavStackStatusHandle::battery_percent() const
{
  std::lock_guard<std::mutex> lock(_pimpl->nav_stack_data->mutex);
  return _pimpl->nav_stack_data->battery_state->percentage;
}

//==============================================================================
std::optional<std::size_t>
NavStackStatusHandle::target_path_waypoint_index() const
{
  std::lock_guard<std::mutex> lock(_pimpl->nav_stack_data->mutex);
  return _pimpl->nav_stack_data->next_path_index;
}

//==============================================================================
double quat_to_yaw(const geometry_msgs::Quaternion& quat)
{
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(quat, tf2_quat);
  tf2::Matrix3x3 tf2_mat(tf2_quat.normalize());

  // ignores pitch and roll
  double yaw;
  double pitch;
  double roll;
  tf2_mat.getEulerYPR(yaw, pitch, roll);
  return yaw;
}

//==============================================================================
bool is_transform_close(
  const geometry_msgs::TransformStamped& first,
  const geometry_msgs::TransformStamped& second)
{
  if (first.header.frame_id != second.header.frame_id || 
      first.child_frame_id != second.child_frame_id)
    return false;

  double elapsed_sec = (second.header.stamp - first.header.stamp).toSec();

  tf2::Vector3 first_pos;
  tf2::Vector3 second_pos;
  tf2::fromMsg(first.transform.translation, first_pos);
  tf2::fromMsg(second.transform.translation, second_pos);
  
  double distance = second_pos.distance(first_pos);
  double speed = abs(distance / elapsed_sec);
  if (speed > 0.01)
    return false;

  tf2::Quaternion first_quat;
  tf2::Quaternion second_quat;
  tf2::fromMsg(first.transform.rotation, first_quat);
  tf2::fromMsg(second.transform.rotation, second_quat);

  double shortest_radian_diff =
    second_quat.normalize().angleShortestPath(first_quat.normalize());
  double turning_speed = abs(shortest_radian_diff / elapsed_sec);
  if (turning_speed > 0.01)
    return false;

  return true;
}

//==============================================================================
} // namespace client
} // namespace free_fleet_ros1
