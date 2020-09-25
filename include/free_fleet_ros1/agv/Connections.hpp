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

#include INCLUDE__FREE_FLEET_ROS1__AGV__CONNECTIONS_HPP
#include INCLUDE__FREE_FLEET_ROS1__AGV__CONNECTIONS_HPP

#include <memory>

#include <rmf_utils/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

namespace free_fleet {
namespace agv {

class Connections
{
public:

  using SharedPtr = std::shared_ptr<Connections>;

  static SharedPtr make(
    const std::string& node_name,
    const std::string& move_base_server_name,
    const std::string& battery_state_topic,
    const std::string& map_frame,
    const std::string& robot_frame,
    int timeout = 10);

private:
  Connections();



};

} // namespace agv
} // namespace free_fleet

#endif // INCLUDE__FREE_FLEET_ROS1__AGV__CONNECTIONS_HPP
