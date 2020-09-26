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

#include <rmf_utils/impl_ptr.hpp>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/BatteryState.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


namespace free_fleet {
namespace ros1 {

class Connections
{
public:

  using SharedPtr = std::shared_ptr<Connections>;

  using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

  static SharedPtr make(
    const std::string& node_name,
    const std::string& move_base_server_name,
    const std::string& battery_state_topic,
    int timeout = 10);

  std::shared_ptr<ros::NodeHandle> node() const;

  std::shared_ptr<MoveBaseClient> move_base_client() const;

  sensor_msgs::BatteryState battery_state() const;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer() const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
  Connections();
};

} // namespace ros1
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET_ROS1__ROS1__CONNECTIONS_HPP
