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

#ifndef SRC__FREE_FLEET_ROS1__CLIENT__NAVSTACKDATA_HPP
#define SRC__FREE_FLEET_ROS1__CLIENT__NAVSTACKDATA_HPP

#include <atomic>
#include <string>
#include <vector>
#include <optional>
#include <shared_mutex>
#include <unordered_map>

#include <sensor_msgs/BatteryState.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/Waypoint.hpp>
#include <free_fleet/messages/RobotMode.hpp>

namespace free_fleet_ros1 {
namespace client {

class NavStackData : public std::enable_shared_from_this<NavStackData>
{
public:

  using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  using ReadLock = std::shared_lock<std::shared_mutex>;
  using WriteLock = std::unique_lock<std::shared_mutex>;

  //============================================================================
  // ROS related items
  std::shared_ptr<ros::NodeHandle> ros_node;

  std::shared_ptr<MoveBaseClient> move_base_client;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer;

  std::shared_ptr<tf2_ros::TransformListener> tf2_listener;

  std::shared_ptr<ros::ServiceClient> set_map_service_client;

  std::unordered_map<std::string, std::shared_ptr<ros::ServiceClient>>
    get_map_service_client_map;

  // TODO(AA): Abstract away the method of obtaining the battery state. This
  // should be done when we migrate to a feature based system.
  ros::Subscriber battery_state_sub;

  //============================================================================
  std::atomic<bool> robot_stopped;

  std::string map_name;

  std::string map_frame;

  std::string robot_frame;

  uint32_t update_frequency;

  uint32_t timeout_sec;

  std::vector<free_fleet::messages::Waypoint> path;

  //============================================================================
  std::optional<std::size_t> next_path_index() const;

  void next_path_index(std::optional<std::size_t> index);

  //============================================================================
  std::optional<sensor_msgs::BatteryState> battery_state() const;

  void battery_state(const sensor_msgs::BatteryState& battery_state);

  //============================================================================
  std::optional<free_fleet::messages::RobotMode> mode() const;

  void mode(const free_fleet::messages::RobotMode& mode);

  //============================================================================
  std::optional<free_fleet::messages::Location> location() const;

  void location(const free_fleet::messages::Location& location);

private:
  
  std::optional<std::size_t> _next_path_index = std::nullopt;

  mutable std::shared_mutex _next_path_index_mutex;

  std::optional<sensor_msgs::BatteryState> _battery_state = std::nullopt;

  mutable std::shared_mutex _battery_state_mutex;
 
  std::optional<free_fleet::messages::RobotMode> _mode = std::nullopt;

  mutable std::shared_mutex _mode_mutex;

  std::optional<free_fleet::messages::Location> _location = std::nullopt;

  mutable std::shared_mutex _loc_mutex;
};

} // client
} // free_fleet_ros1

#endif // SRC__FREE_FLEET_ROS1__CLIENT__NAVSTACKDATA_HPP
