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

#ifndef INCLUDE__FREE_FLEET_ROS1__CLIENT__NAVSTACKCOMMANDHANDLE_HPP
#define INCLUDE__FREE_FLEET_ROS1__CLIENT__NAVSTACKCOMMANDHANDLE_HPP

#include <memory>
#include <string>

#include <rmf_utils/impl_ptr.hpp>
#include <free_fleet/client/CommandHandle.hpp>

namespace free_fleet_ros1 {
namespace client {

class NavStackCommandHandle : public free_fleet::CommandHandle
{
public:

  void relocalize(
    const free_fleet::messages::Location& location,
    RequestCompleted relocalization_finished_callback) final;

  void follow_new_path(
      const std::vector<free_fleet::messages::Waypoint>& waypoints,
      RequestCompleted path_finished_callback) final;

  void stop() final;

  void resume() final;

  void dock(
    const std::string& dock_name,
    RequestCompleted docking_finished_callback) final;

  class Implementation;
private:
  NavStackCommandHandle();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace client 
} // namespace free_fleet_ros1

#endif // INCLUDE__FREE_FLEET_ROS1__CLIENT__NAVSTACKCOMMANDHANDLE_HPP
