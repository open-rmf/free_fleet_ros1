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

#include <map>
#include <string>
#include <iostream>

#include <free_fleet/agv/Client.hpp>

#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

#include <free_fleet_ros1/ros1/Connections.hpp>
#include <free_fleet_ros1/agv/StatusHandle.hpp>
#include <free_fleet_ros1/agv/NavStackCommandHandle.hpp>

template<typename T>
void get_param(const ros::NodeHandle& node, const std::string& key, T& param)
{
  if (node.getParam(key, param))
  {
    ROS_INFO("Found [%s] on the parameter server. Setting [%s] to [%s].",
        key.c_str(), key.c_str(), (std::to_string(param)).c_str());
  }
}

void get_param(
  const ros::NodeHandle& node,
  const std::string& key,
  std::string& param)
{
  if (node.getParam(key, param))
  {
    ROS_INFO("Found [%s] on the parameter server. Setting [%s] to [%s].",
        key.c_str(), key.c_str(), param.c_str());
  }
}

void get_param(
  const ros::NodeHandle& node,
  const std::string& key,
  std::map<std::string, std::string>& param)
{
  if (node.getParam(key, param))
  {
    for (const auto& it : param)
    {
      ROS_INFO("Found map name [%s] with get_map service [%s].",
        it.first.c_str(), it.second.c_str());
    }
  }
}

int main(int argc, char** argv)
{
  std::string node_name = "free_fleet_ros1_client_node";
  ros::init(argc, argv, node_name);

  std::string fleet_name = "fleet_name";
  std::string robot_name = "robot_name";
  std::string robot_model = "robot_model";
  std::string initial_map_name = "L1";
  int dds_domain = 24;
  std::string move_base_server = "move_base";
  std::string set_map_server = "set_map";
  std::map<std::string, std::string> get_map_servers;
  std::string battery_state_topic = "/battery_state";
  std::string map_frame = "map";
  std::string robot_frame = "base_footprint";
  int frequency = 10;

  ros::NodeHandle node_private_ns("~");
  get_param(node_private_ns, "fleet_name", fleet_name);
  get_param(node_private_ns, "robot_name", robot_name);
  get_param(node_private_ns, "robot_model", robot_model);
  get_param(node_private_ns, "initial_map_name", initial_map_name);
  get_param(node_private_ns, "dds_domain", dds_domain);
  get_param(node_private_ns, "node_name", node_name);
  get_param(node_private_ns, "move_base_server", move_base_server);
  get_param(node_private_ns, "set_map_server", set_map_server);
  get_param(node_private_ns, "get_map_servers", get_map_servers);
  get_param(node_private_ns, "battery_state_topic", battery_state_topic);
  get_param(node_private_ns, "map_frame", map_frame);
  get_param(node_private_ns, "robot_frame", robot_frame);
  get_param(node_private_ns, "frequency", frequency);

  auto middleware =
    free_fleet::cyclonedds::CycloneDDSMiddleware::make_client(
      dds_domain, fleet_name);

  free_fleet_ros1::ros1::Connections::MapNameServiceMap get_map_server_map;
  for (const auto& it : get_map_servers)
  {
    get_map_server_map[it.first] = it.second;
  }

  auto connections =
    free_fleet_ros1::ros1::Connections::make(
      node_name,
      move_base_server,
      set_map_server,
      get_map_server_map,
      battery_state_topic,
      initial_map_name);
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
      robot_name,
      robot_model,
      command_handle,
      status_handle,
      middleware);
  client->start(frequency);

  return 0;
}
