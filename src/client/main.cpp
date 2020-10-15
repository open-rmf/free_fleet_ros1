/*
 * Copyright (C) 2002 Open Source Robotics Foundation
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

#include <string>
#include <iostream>

#include <free_fleet/agv/Client.hpp>

#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

#include <free_fleet_ros1/ros1/Connections.hpp>
#include <free_fleet_ros1/agv/StatusHandle.hpp>
#include <free_fleet_ros1/agv/NavStackCommandHandle.hpp>

int main(int argc, char** argv)
{
  std::string robot_name = "test_robot";
  std::string robot_model = "test_model";
  std::string fleet_name = "test_fleet";
  int dds_domain = 24;
  std::string node_name = "test_client_node";
  std::string move_base_server_name = "move_base";
  std::string battery_state_topic = "/battery_state";
  std::string level_name = "L1";
  std::string map_frame = "/map";
  std::string robot_frame = "/base_footprint";

  ros::init(argc, argv, node_name);

  auto middleware =
    free_fleet::cyclonedds::CycloneDDSMiddleware::make_client(
      dds_domain, fleet_name);

  auto connections =
    free_fleet_ros1::ros1::Connections::make(
      node_name, move_base_server_name, battery_state_topic, level_name);
  if (!connections)
  {
    ROS_ERROR("Failed to start client.");
    return 1;
  }

  auto status_handle =
    free_fleet_ros1::agv::StatusHandle::make(
      connections, map_frame, robot_frame);

  auto command_handle =
    free_fleet_ros1::agv::NavStackCommandHandle::make(connections, map_frame);

  auto client =
    free_fleet::agv::Client::make(
      robot_name, robot_model, command_handle, status_handle, middleware);

  return 0;
}
