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
  std::shared_ptr<NavStackData> nav_stack_data)
-> std::shared_ptr<NavStackStatusHandle>
{
  std::unique_ptr<ros::Rate> update_rate(
    new ros::Rate(nav_stack_data->update_frequency));

  std::shared_ptr<NavStackStatusHandle> status_handle(
    new NavStackStatusHandle());
  status_handle->_pimpl->nav_stack_data = std::move(nav_stack_data);
  status_handle->_pimpl->update_rate = std::move(update_rate);

  uint32_t count = 0;
  auto prev_t = std::chrono::steady_clock::now();
  while (!status_handle->_pimpl->initialized() &&
      count++ < status_handle->_pimpl->nav_stack_data->timeout_sec)
  {
    prev_t = std::chrono::steady_clock::now();
    status_handle->_pimpl->update_transforms();
    status_handle->_pimpl->update_mode();

    ffinfo << "Waiting for incoming transforms and battery states...\n";
    std::this_thread::sleep_until(prev_t + std::chrono::seconds(1));
  }

  if (!status_handle->_pimpl->initialized())
  {
    fferr << "Timed out trying to initialize status handle for client.\n";
    return nullptr;
  }

  return status_handle;
}

//==============================================================================
bool NavStackStatusHandle::Implementation::initialized() const
{
  if (previous_transform.has_value() &&
      current_transform.has_value() &&
      nav_stack_data->battery_state().has_value() &&
      nav_stack_data->mode().has_value() &&
      nav_stack_data->location().has_value())
  {
    return true;
  }
  return false;
}

//==============================================================================
void NavStackStatusHandle::Implementation::update_transforms()
{
  try
  {
    geometry_msgs::TransformStamped tmp_transform =
      nav_stack_data->tf2_buffer->lookupTransform(
        nav_stack_data->map_frame,
        nav_stack_data->robot_frame,
        ros::Time(0));
    previous_transform = current_transform;
    current_transform = tmp_transform;
  }
  catch (tf2::TransformException &ex)
  {
    ffwarn << "Could not retrieve transform between from robot frame ["
      << nav_stack_data->robot_frame << "] to map frame ["
      << nav_stack_data->map_frame << "]: "
      << ex.what() << ".\n";
  }
}

//==============================================================================
void NavStackStatusHandle::Implementation::update_mode()
{
  auto battery_state = nav_stack_data->battery_state();
  if (!battery_state.has_value())
  {
    ffinfo << "Waiting for battery state before RobotMode can get updated.\n";
    return;
  }

  if (!previous_transform.has_value() ||
      !current_transform.has_value())
  {
    ffinfo << "Waiting for more transforms before RobotMode can get updated.\n";
    return;
  }

  using BatteryState = sensor_msgs::BatteryState;
  using RobotMode = free_fleet::messages::RobotMode;

  if (battery_state->power_supply_status ==
      BatteryState::POWER_SUPPLY_STATUS_CHARGING)
  {
    nav_stack_data->mode(RobotMode(RobotMode::Mode::Charging));
  }
  else if (!is_transform_close(
      current_transform.value(), previous_transform.value()))
  {
    nav_stack_data->mode(RobotMode(RobotMode::Mode::Moving));
  }
  else if (nav_stack_data->robot_stopped)
  {
    nav_stack_data->mode(RobotMode(RobotMode::Mode::Paused));
  }
  else
  {
    nav_stack_data->mode(RobotMode(RobotMode::Mode::Idle));
  }
}

//==============================================================================
void NavStackStatusHandle::Implementation::update_thread_fn()
{
  while (nav_stack_data && nav_stack_data->ros_node->ok())
  {
    update_rate->sleep();
    update_transforms();
    update_mode();

    // Updating location
    using Location = free_fleet::messages::Location;
    nav_stack_data->location(
      Location(
        nav_stack_data->map_name,
        {current_transform->transform.translation.x,
        current_transform->transform.translation.y},
        quat_to_yaw(current_transform->transform.rotation)));
  }
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
  auto loc = _pimpl->nav_stack_data->location();
  assert(loc.has_value());
  return loc.value();
}

//==============================================================================
free_fleet::messages::RobotMode NavStackStatusHandle::mode() const
{
  auto mode = _pimpl->nav_stack_data->mode();
  assert(mode.has_value());
  return mode.value();
}

//==============================================================================
double NavStackStatusHandle::battery_percent() const
{
  auto battery_state = _pimpl->nav_stack_data->battery_state();
  assert(battery_state.has_value());
  return battery_state->percentage;
}

//==============================================================================
std::optional<std::size_t>
NavStackStatusHandle::target_path_waypoint_index() const
{
  return _pimpl->nav_stack_data->next_path_index();
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
