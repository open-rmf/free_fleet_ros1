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

#include <mutex>
#include <atomic>
#include <string>
#include <vector>
#include <optional>
#include <unordered_map>

#include <sensor_msgs/BatteryState.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet/messages/Waypoint.hpp>

namespace free_fleet_ros1 {
namespace client {

class NavStackData : public std::enable_shared_from_this<NavStackData>
{
public:

  using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  std::shared_ptr<ros::NodeHandle> ros_node;

  std::shared_ptr<MoveBaseClient> move_base_client;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer;

  std::shared_ptr<tf2_ros::TransformListener> tf2_listener;

  std::shared_ptr<ros::ServiceClient> set_map_service_client;

  std::unordered_map<std::string, std::shared_ptr<ros::ServiceClient>>
    get_map_service_client_map;

  ros::Subscriber battery_state_sub;

  std::optional<sensor_msgs::BatteryState> battery_state = std::nullopt;

  std::atomic<bool> robot_stopped;

  std::string map_name;

  std::string map_frame;

  std::string robot_frame;

  int update_frequency;

  int timeout_sec;
  
  std::optional<free_fleet::messages::Location> location = std::nullopt;
 
  std::optional<free_fleet::messages::RobotMode> mode = std::nullopt;

  std::vector<free_fleet::messages::Waypoint> path;

  std::optional<std::size_t> next_path_index;

  mutable std::mutex mutex;
};

} // client
} // free_fleet_ros1

#endif // SRC__FREE_FLEET_ROS1__CLIENT__NAVSTACKDATA_HPP
