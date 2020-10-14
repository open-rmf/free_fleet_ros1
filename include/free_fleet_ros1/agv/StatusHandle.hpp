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

#ifndef INCLUDE__FREE_FLEET_ROS1__AGV__STATUSHANDLE_HPP
#define INCLUDE__FREE_FLEET_ROS1__AGV__STATUSHANDLE_HPP

#include <rmf_utils/impl_ptr.hpp>

#include <free_fleet/agv/StatusHandle.hpp>
#include <free_fleet_ros1/ros1/Connections.hpp>

namespace free_fleet_ros1 {
namespace agv {

class StatusHandle : public free_fleet::agv::StatusHandle
{
public:

  using SharedPtr = std::shared_ptr<StatusHandle>;

  static SharedPtr make(
    ros1::Connections::SharedPtr connections,
    std::string map_frame,
    std::string robot_frame);

  free_fleet::messages::Location location() const final;

  free_fleet::messages::RobotMode mode() const final;

  double battery_percent() const final;

  std::vector<free_fleet::messages::Location> path() const final;

  class Implementation;
private:
  StatusHandle();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace free_fleet_ros1

#endif // INCLUDE__FREE_FLEET_ROS1__AGV__STATUSHANDLE_HPP
