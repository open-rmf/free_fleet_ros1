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

#ifndef INCLUDE__FREE_FLEET_ROS1__CLIENT__NAVSTACKSTATUSHANDLE_HPP
#define INCLUDE__FREE_FLEET_ROS1__CLIENT__NAVSTACKSTATUSHANDLE_HPP

#include <memory>
#include <string>

#include <rmf_utils/impl_ptr.hpp>
#include <free_fleet/client/StatusHandle.hpp>

namespace free_fleet_ros1 {
namespace client {

class NavStackStatusHandle : public free_fleet::client::StatusHandle
{
public:

  rmf_traffic::Time time() const final;

  free_fleet::messages::Location location() const final;

  free_fleet::messages::RobotMode mode() const final;

  double battery_percent() const final;

  std::optional<std::size_t> target_path_waypoint_index() const final;

  class Implementation;
private:
  NavStackStatusHandle();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace client 
} // namespace free_fleet_ros1

#endif // INCLUDE__FREE_FLEET_ROS1__CLIENT__NAVSTACKSTATUSHANDLE_HPP
