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

#ifndef INCLUDE__FREE_FLEET_ROS1__AGV__NAVSTACKCOMMANDHANDLE_HPP
#define INCLUDE__FREE_FLEET_ROS1__AGV__NAVSTACKCOMMANDHANDLE_HPP

#include <memory>

#include <ros/ros.h>

#include <rmf_utils/impl_ptr.hpp>
#include <rmf_traffic/agv/Planner.hpp>

#include <free_fleet/agv/CommandHandle.hpp>

namespace free_fleet {
namespace agv {

class NavStackCommandHandle : public CommandHandle
{
public:

  using SharedPtr = std::shared_ptr<NavStackCommandHandle>;

  static SharedPtr make(
    std::shared_ptr<ros::NodeHandle> node,
    const std::string& move_base_server_name,
    int timeout = 10);

  ~NavStackCommandHandle();

  void follow_new_path(
    const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
    RequestCompleted path_finished_callback) final;

  void stop() final;

  void resume() final;

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
  NavStackCommandHandle();
};

} // namespace ros1
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET_ROS1__AGV__NAVSTACKCOMMANDHANDLE_HPP
