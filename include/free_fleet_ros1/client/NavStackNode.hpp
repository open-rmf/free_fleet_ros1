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

#ifndef INCLUDE__FREE_FLEET_ROS1__CLIENT__NAVSTACKNODE_HPP
#define INCLUDE__FREE_FLEET_ROS1__CLIENT__NAVSTACKNODE_HPP

#include <memory>
#include <string>
#include <unordered_map>

#include <rmf_utils/impl_ptr.hpp>
#include <free_fleet_ros1/client/NavStackStatusHandle.hpp>
// #include <free_fleet_ros1/client/NavStackCommandHandle.hpp>

namespace free_fleet_ros1 {
namespace client {

class NavStackNode
{
public:

  /// Unordered map where the map name is key while the GetMap service name is
  /// the value.
  using MapNameServiceMap = std::unordered_map<std::string, std::string>;

  /// Static make function to set up all the connections needed for the free
  /// fleet Client to work with the ROS1 navigation stack.
  static std::shared_ptr<NavStackNode> make(
    const std::string& node_name,
    const std::string& move_base_server_name,
    const std::string& set_map_server_name,
    const MapNameServiceMap& map_services,
    const std::string& battery_state_topic,
    const std::string& initial_map_name,
    const std::string& map_frame,
    const std::string& robot_frame,
    int update_frequency = 10,
    int timeout_sec = 10);

  /// Returns a shared CommandHandle used by the free fleet Client.
  // std::shared_ptr<NavStackCommandHandle> command_handle() const;

  /// Returns a shared StatusHandle used by the free fleet Client. If a
  /// StatusHandle has not been requested before, this will create a new one,
  /// otherwise it will return a shared pointer to the previously created
  /// StatusHandle. If the initialization of a new StatusHandle fails, this
  /// returns a nullptr.
  std::shared_ptr<NavStackStatusHandle> status_handle();

  class Implementation;
private:
  NavStackNode();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace client
} // namespace free_fleet_ros1

#endif // INCLUDE__FREE_FLEET_ROS1__CLIENT__NAVSTACKNODE_HPP
