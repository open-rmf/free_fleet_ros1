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

#include <mutex>
#include <thread>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>

#include <free_fleet_ros1/client/Connections.hpp>

namespace free_fleet_ros1 {
namespace client {

//==============================================================================
class Connections::Implementation
{
public:

  Implementation()
  : robot_stopped(false)
  {
  }

  Implementation(const Implementation&)
  : robot_stopped(false)
  {
    // Empty copy constructor, only needed during construction of impl_ptr
    // All members will be initialized or assigned during runtime.
  }

  ~Implementation()
  {
    if (spin_thread.joinable())
      spin_thread.join();
  }

  void thread_fn()
  {
    ros::spin();
  }

  void battery_state_callback_fn(const sensor_msgs::BatteryState& msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    battery_state = msg;
  }

  void start(const std::string& battery_state_topic)
  {
    battery_state_sub = node->subscribe(
      battery_state_topic, 1,
      &Implementation::battery_state_callback_fn, 
      this);
    spin_thread = std::thread(std::bind(&Implementation::thread_fn, this));
  }

  std::shared_ptr<ros::NodeHandle> node;

  std::shared_ptr<MoveBaseClient> move_base_client;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer;

  std::shared_ptr<tf2_ros::TransformListener> tf2_listener;

  std::shared_ptr<ros::ServiceClient> set_map_service_client;

  std::unordered_map<std::string, std::shared_ptr<ros::ServiceClient>>
    get_map_service_client_map;

  ros::Subscriber battery_state_sub;

  sensor_msgs::BatteryState battery_state;

  std::atomic<bool> robot_stopped;

  std::string map_name;

  std::vector<free_fleet::messages::Waypoint> path;

  std::size_t next_path_index;

  mutable std::mutex mutex;

  std::thread spin_thread;
};

//==============================================================================
std::shared_ptr<Connections> Connections::make(
  const std::string& node_name,
  const std::string& move_base_server_name,
  const std::string& set_map_server_name,
  const MapNameServiceMap& map_services,
  const std::string& battery_state_topic,
  const std::string& initial_map_name,
  int timeout)
{
  std::shared_ptr<Connections> connections(new Connections());

  std::shared_ptr<ros::NodeHandle> node(new ros::NodeHandle(node_name));
  std::shared_ptr<MoveBaseClient> move_base_client(
    new MoveBaseClient(move_base_server_name, true));
  if (!move_base_client || 
    !move_base_client->waitForServer(ros::Duration(timeout)))
  {
    ROS_ERROR("Timed out waiting for MoveBase action service: %s",
        move_base_server_name.c_str());
    return nullptr;
  }

  std::shared_ptr<tf2_ros::Buffer> buffer(new tf2_ros::Buffer());
  std::shared_ptr<tf2_ros::TransformListener> listener(
    new tf2_ros::TransformListener(*buffer));

  auto set_map_service_client =
    std::make_shared<ros::ServiceClient>(
      node->serviceClient<nav_msgs::SetMap>(set_map_server_name, true));
  if (!set_map_service_client ||
    !set_map_service_client->waitForExistence(ros::Duration(timeout)))
  {
    ROS_ERROR("Timed out waiting for SetMAp service: %s",
      set_map_server_name.c_str());
    return nullptr;
  }

  std::unordered_map<std::string, std::shared_ptr<ros::ServiceClient>>
    get_map_service_client_map;
  for (const auto& m : map_services)
  {
    auto get_map_service_client =
      std::make_shared<ros::ServiceClient>(
        node->serviceClient<nav_msgs::GetMap>(m.second, true));
    if (!get_map_service_client ||
      !get_map_service_client->waitForExistence(ros::Duration(timeout)))
    {
      ROS_ERROR("Timed out waiting for GetMap service: %s", m.second.c_str());
      return nullptr;
    }

    get_map_service_client_map[m.first] = std::move(get_map_service_client);
  }

  auto it = get_map_service_client_map.find(initial_map_name);
  if (it == get_map_service_client_map.end())
  {
    ROS_ERROR(
      "Map service for initial map [%s] not provided.",
      initial_map_name.c_str());
    return nullptr;
  }

  if (!node || !buffer || !listener)
  {
    ROS_ERROR("Unable to initialize ROS 1 connections.");
    return nullptr;
  }

  connections->_pimpl->node = std::move(node);
  connections->_pimpl->move_base_client = std::move(move_base_client);
  connections->_pimpl->tf2_buffer = std::move(buffer);
  connections->_pimpl->tf2_listener = std::move(listener);
  connections->_pimpl->set_map_service_client =
    std::move(set_map_service_client);
  connections->_pimpl->get_map_service_client_map =
    std::move(get_map_service_client_map);
  connections->_pimpl->map_name = std::move(initial_map_name);
  connections->_pimpl->start(battery_state_topic);
  return connections;
}

//==============================================================================
Connections::Connections()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
}

//==============================================================================
auto Connections::node() const -> std::shared_ptr<ros::NodeHandle>
{
  return _pimpl->node;
}

//==============================================================================
auto Connections::move_base_client() const -> std::shared_ptr<MoveBaseClient>
{
  return _pimpl->move_base_client;
}

//==============================================================================
auto Connections::tf2_buffer() const -> std::shared_ptr<tf2_ros::Buffer>
{
  return _pimpl->tf2_buffer;
}

//==============================================================================
auto Connections::set_map_service_client() const
  -> std::shared_ptr<ros::ServiceClient>
{
  return _pimpl->set_map_service_client;
}

//==============================================================================
auto Connections::battery_state() const -> sensor_msgs::BatteryState
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  return _pimpl->battery_state;
}

//==============================================================================
auto Connections::maps() const -> std::vector<std::string>
{
  std::vector<std::string> maps;
  for (const auto it : _pimpl->get_map_service_client_map)
  {
    maps.push_back(it.first);
  }
  return maps;
}

//==============================================================================
auto Connections::get_map_client(const std::string& map_name) const
-> std::shared_ptr<ros::ServiceClient>
{
  auto it = _pimpl->get_map_service_client_map.find(map_name);
  if (it == _pimpl->get_map_service_client_map.end())
    return nullptr;

  return it->second;
}

//==============================================================================
bool Connections::robot_stopped() const
{
  return _pimpl->robot_stopped;
}

//==============================================================================
void Connections::robot_stopped(bool new_robot_stopped_state)
{
  _pimpl->robot_stopped = new_robot_stopped_state;
}

//==============================================================================
std::string Connections::map_name() const
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  return _pimpl->map_name;
}

//==============================================================================
void Connections::map_name(const std::string& new_map_name)
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  _pimpl->map_name = new_map_name;
}

//==============================================================================
std::vector<free_fleet::messages::Waypoint> Connections::path() const
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  return _pimpl->path;
}

//==============================================================================
void Connections::path(
  const std::vector<free_fleet::messages::Waypoint>& new_path)
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  _pimpl->path = new_path;
  _pimpl->next_path_index = 0;
}

//==============================================================================
std::size_t Connections::next_path_index() const
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  return _pimpl->next_path_index;
}

//==============================================================================
void Connections::next_path_index(std::size_t index)
{
  std::lock_guard<std::mutex> lock(_pimpl->mutex);
  if (index >= _pimpl->path.size())
  {
    ROS_ERROR("Next path index is out of range of the path.");
    return;
  }
  _pimpl->next_path_index = index;
}

//==============================================================================
} // namespace ros1
} // namespace free_fleet_ros1
