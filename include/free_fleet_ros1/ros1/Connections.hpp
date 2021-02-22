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

#ifndef INCLUDE__FREE_FLEET_ROS1__ROS1__CONNECTIONS_HPP
#define INCLUDE__FREE_FLEET_ROS1__ROS1__CONNECTIONS_HPP

#include <memory>
#include <unordered_map>

#include <rmf_utils/impl_ptr.hpp>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/BatteryState.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/Waypoint.hpp>

namespace free_fleet_ros1 {
namespace ros1 {

class Connections
{
public:

  using SharedPtr = std::shared_ptr<Connections>;

  using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  /// Unordered map where the map name is key while the GetMap service name is
  /// the value.
  using MapNameServiceMap = std::unordered_map<std::string, std::string>;

  /// Make function for all ROS 1 connections.
  ///
  /// \param[in] node_name
  /// \param[in] move_base_server_name
  /// \param[in] set_map_server_name
  /// \param[in] map_services
  /// \param[in] battery_state_topic
  /// \param[in] level_name
  /// \param[in] timeout
  /// \return
  static SharedPtr make(
    const std::string& node_name,
    const std::string& move_base_server_name,
    const std::string& set_map_server_name,
    const MapNameServiceMap& map_services,
    const std::string& battery_state_topic,
    const std::string& level_name,
    int timeout = 10);

  /// Gets a shared pointer to the ROS node.
  std::shared_ptr<ros::NodeHandle> node() const;

  /// Gets a shared pointer to the MoveBase action client.
  std::shared_ptr<MoveBaseClient> move_base_client() const;

  /// Gets a shared pointer to the tf2 buffer.
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer() const;

  /// Gets a shared pointer to the SetMap service client.
  std::shared_ptr<ros::ServiceClient> set_map_service_client() const;

  /// Gets the most recent battery state.
  sensor_msgs::BatteryState battery_state() const;

  /// Gets the names of all the maps available.
  std::vector<std::string> maps() const;

  /// Gets the GetMap client corresponding to the map name. Returns a nullptr if
  /// there does not exist a client for that map.
  std::shared_ptr<ros::ServiceClient> get_map_client(
    const std::string& map_name) const;

  /// Checks if the robot has stopped.
  bool stopped() const;

  /// Modify the robots stopped state.
  void stopped(bool new_stopped_state);

  /// Current level name.
  std::string level_name() const;

  /// Sets the current level name.
  void level_name(const std::string& new_level_name);

  /// Current path that the robot is on.
  std::vector<free_fleet::messages::Waypoint> path() const;

  /// Sets the current path of the robot.
  void path(const std::vector<free_fleet::messages::Waypoint>& new_path);

  /// Gets the next path index that the robot is heading towards.
  std::size_t next_path_index() const;

  /// Sets the next path index.
  void next_path_index(std::size_t index);

  class Implementation;
private:
  Connections();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace ros1
} // namespace free_fleet_ros1

#endif // INCLUDE__FREE_FLEET_ROS1__ROS1__CONNECTIONS_HPP
