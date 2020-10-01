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

#include <free_fleet_ros1/ros1/Connections.hpp>

namespace free_fleet {
namespace ros1 {

//==============================================================================

class Connections::Implementation
{
public:

  Implementation()
  {}

  Implementation(const Implementation&)
  {
    // Empty copy constructor, only needed during construction of impl_ptr
    // All members will be initialized or assigned during runtime.
  }

  ~Implementation()
  {
    if (_spin_thread.joinable())
      _spin_thread.join();
  }

  void thread_fn()
  {
    ros::spin();
  }

  void battery_state_callback_fn(const sensor_msgs::BatteryState& msg)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    _battery_state = msg;
  }

  void start(const std::string& battery_state_topic)
  {
    _battery_state_sub =
      _node->subscribe(
        battery_state_topic, 1, &Implementation::battery_state_callback_fn, 
        this);
    _spin_thread = std::thread(std::bind(&Implementation::thread_fn, this));
  }

  std::shared_ptr<ros::NodeHandle> _node;
  std::shared_ptr<MoveBaseClient> _move_base_client;

  std::shared_ptr<tf2_ros::Buffer> _tf2_buffer;
  std::shared_ptr<tf2_ros::TransformListener> _tf2_listener;
  
  ros::Subscriber _battery_state_sub;
  sensor_msgs::BatteryState _battery_state;

  mutable std::mutex _mutex;
  std::thread _spin_thread;
};

//==============================================================================
Connections::SharedPtr Connections::make(
  const std::string& node_name,
  const std::string& move_base_server_name,
  const std::string& battery_state_topic,
  int timeout)
{
  Connections::SharedPtr connections(new Connections());

  std::shared_ptr<ros::NodeHandle> node(new ros::NodeHandle(node_name));
  std::shared_ptr<MoveBaseClient> move_base_client(
    new MoveBaseClient(move_base_server_name, true));
  if (!move_base_client ||
    !move_base_client->waitForServer(ros::Duration(timeout)))
  {
    ROS_ERROR("Timed out waiting for action server: %s",
        move_base_server_name.c_str());
    return nullptr;
  }

  std::shared_ptr<tf2_ros::Buffer> buffer(new tf2_ros::Buffer());
  std::shared_ptr<tf2_ros::TransformListener> listener(
    new tf2_ros::TransformListener(*buffer));

  if (!node || !buffer || !listener)
  {
    ROS_ERROR("Unable to initialize ROS 1 connections.");
    return nullptr;
  }

  connections->_pimpl->_node = std::move(node);
  connections->_pimpl->_move_base_client = std::move(move_base_client);
  connections->_pimpl->_tf2_buffer = std::move(buffer);
  connections->_pimpl->_tf2_listener = std::move(listener);
  connections->_pimpl->start(battery_state_topic);
  return connections;
}

//==============================================================================
Connections::Connections()
: _pimpl(rmf_utils::make_impl<Implementation>())
{}

//==============================================================================
auto Connections::node() const -> std::shared_ptr<ros::NodeHandle>
{
  return _pimpl->_node;
}

//==============================================================================
auto Connections::move_base_client() const -> std::shared_ptr<MoveBaseClient>
{
  return _pimpl->_move_base_client;
}

//==============================================================================
auto Connections::tf2_buffer() const -> std::shared_ptr<tf2_ros::Buffer>
{
  return _pimpl->_tf2_buffer;
}

//==============================================================================
auto Connections::battery_state() const -> sensor_msgs::BatteryState
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  return _pimpl->_battery_state;
}

//==============================================================================
} // namespace ros1
} // namespace free_fleet
