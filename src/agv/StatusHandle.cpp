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

#include <mutex>
#include <thread>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <free_fleet_ros1/agv/StatusHandle.hpp>

namespace free_fleet_ros1 {
namespace agv {

//==============================================================================
class StatusHandle::Implementation
{
public:
  Implementation()
  {}

  Implementation(const Implementation&)
  {
    // Empty copy constructor, only needed during construction of impl_ptr
    // All members will be initialized or assigned during runtime.
  }

  ~Implementation()
  {
    if (_thread.joinable())
      _thread.join();
  }

  double quat_to_yaw(const geometry_msgs::Quaternion& quat) const
  {
    tf2::Quaternion tf2_quat;
    tf2::fromMsg(quat, tf2_quat);
    tf2::Matrix3x3 tf2_mat(tf2_quat);
    
    // ignores pitch and roll, but the api call is so nice though
    double yaw;
    double pitch;
    double roll;
    tf2_mat.getEulerYPR(yaw, pitch, roll);
    return yaw;
  }

  bool is_transform_close(
    const geometry_msgs::TransformStamped& first,
    const geometry_msgs::TransformStamped& second) const
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

    double first_yaw = quat_to_yaw(first.transform.rotation);
    double second_yaw = quat_to_yaw(second.transform.rotation);
    double turning_speed = abs((second_yaw - first_yaw) / elapsed_sec);
    if (turning_speed > 0.01)
      return false;

    return true;
  }

  void _update_thread_fn()
  {
    while (_node->ok())
    {
      _update_rate->sleep();

      try {
        geometry_msgs::TransformStamped tmp_transform =
            _tf2_buffer->lookupTransform(
              _map_frame, _robot_frame, ros::Time(0));
        _previous_transform = _current_transform;
        _current_transform = tmp_transform;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
      }

      sensor_msgs::BatteryState battery_state = _connections->battery_state();

      free_fleet::messages::RobotMode robot_mode;
      if (battery_state.power_supply_status == battery_state.POWER_SUPPLY_STATUS_CHARGING)
        robot_mode.mode = robot_mode.MODE_CHARGING;
      else if (!is_transform_close(_current_transform, _previous_transform))
        robot_mode.mode = robot_mode.MODE_MOVING;
      else if (_connections->stopped())
        robot_mode.mode = robot_mode.MODE_PAUSED;
      else
        robot_mode.mode = robot_mode.MODE_IDLE;

      std::lock_guard<std::mutex> lock(_mutex);

      _location.sec = _current_transform.header.stamp.sec;
      _location.nanosec = _current_transform.header.stamp.nsec;
      _location.x = _current_transform.transform.translation.x;
      _location.y = _current_transform.transform.translation.y;
      _location.yaw = quat_to_yaw(_current_transform.transform.rotation);
      _location.level_name = _connections->level_name();

      _mode.mode = robot_mode.mode;

      _battery_percent = battery_state.percentage;
    }
  }

  void _start()
  {
    ROS_INFO("Starting spin thread.");
    _thread =
      std::thread(std::bind(&Implementation::_update_thread_fn, this));
  }

  ros1::Connections::SharedPtr _connections;
  std::shared_ptr<ros::NodeHandle> _node;
  std::shared_ptr<tf2_ros::Buffer> _tf2_buffer;
  std::unique_ptr<ros::Rate> _update_rate;

  std::string _map_frame;
  std::string _robot_frame;

  geometry_msgs::TransformStamped _previous_transform;
  geometry_msgs::TransformStamped _current_transform;

  free_fleet::messages::Location _location;
  free_fleet::messages::RobotMode _mode;
  double _battery_percent;

  std::thread _thread;
  mutable std::mutex _mutex;
};

//==============================================================================
StatusHandle::SharedPtr StatusHandle::make(
  ros1::Connections::SharedPtr connections,
  std::string map_frame,
  std::string robot_frame)
{
  StatusHandle::SharedPtr status_handle(new StatusHandle());

  status_handle->_pimpl->_connections = std::move(connections);
  status_handle->_pimpl->_node = status_handle->_pimpl->_connections->node();
  status_handle->_pimpl->_tf2_buffer =
    status_handle->_pimpl->_connections->tf2_buffer();
  status_handle->_pimpl->_update_rate.reset(new ros::Rate(10));
  status_handle->_pimpl->_map_frame = std::move(map_frame);
  status_handle->_pimpl->_robot_frame = std::move(robot_frame);
  return status_handle;
}

//==============================================================================
StatusHandle::StatusHandle()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
free_fleet::messages::Location StatusHandle::location() const
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  return _pimpl->_location;
}

//==============================================================================
free_fleet::messages::RobotMode StatusHandle::mode() const
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  return _pimpl->_mode;
}

//==============================================================================
double StatusHandle::battery_percent() const
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  return _pimpl->_battery_percent;
}

//==============================================================================
std::vector<free_fleet::messages::Location> StatusHandle::path() const
{
  return _pimpl->_connections->path();
}

//==============================================================================
} // namespace agv
} // namespace free_fleet_ros1
