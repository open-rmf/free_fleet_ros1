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

#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/BatteryState.h>

#include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet/Console.hpp>
#include <free_fleet_ros1/client/NavStackNode.hpp>

#include "NavStackData.hpp"
#include "internal_NavStackStatusHandle.hpp"
// #include "internal_NavStackCommandHandle.hpp"

namespace free_fleet_ros1 {
namespace client {

//==============================================================================
class NavStackNode::Implementation
{
public:

  Implementation()
  {
  }

  Implementation(const Implementation&)
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

  void start()
  {
    spin_thread = std::thread(std::bind(&Implementation::thread_fn, this));
  }

  std::shared_ptr<NavStackData> data;

  std::shared_ptr<NavStackStatusHandle> nav_stack_status_handle;

  // std::shared_ptr<NavStackCommandHandle> nav_stack_command_handle;

  std::thread spin_thread;
};

//==============================================================================
std::shared_ptr<NavStackNode> NavStackNode::make(
  const std::string& node_name,
  const std::string& move_base_server_name,
  const std::string& set_map_server_name,
  const MapNameServiceMap& map_services,
  const std::string& battery_state_topic,
  const std::string& initial_map_name,
  const std::string& map_frame,
  const std::string& robot_frame,
  uint32_t update_frequency,
  uint32_t timeout_sec)
{
  std::shared_ptr<NavStackData> data(new NavStackData());
  data->ros_node =
    std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle(node_name));

  using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  data->move_base_client = std::shared_ptr<MoveBaseClient>(
    new MoveBaseClient(move_base_server_name, true));
  if (!data->move_base_client || 
    !data->move_base_client->waitForServer(ros::Duration(timeout_sec)))
  {
    fferr << "Timed out waiting for MoveBase action service: "
      << move_base_server_name << "\n";
    return nullptr;
  }

  data->tf2_buffer = std::shared_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer());
  data->tf2_listener =
    std::shared_ptr<tf2_ros::TransformListener>(
      new tf2_ros::TransformListener(*data->tf2_buffer));

  data->set_map_service_client =
    std::make_shared<ros::ServiceClient>(
      data->ros_node->serviceClient<nav_msgs::SetMap>(
        set_map_server_name, true));
  if (!data->set_map_service_client ||
    !data->set_map_service_client->waitForExistence(ros::Duration(timeout_sec)))
  {
    fferr << "Timed out waiting for SetMAp service: "
      << set_map_server_name << "\n";
    return nullptr;
  }

  for (const auto& m : map_services)
  {
    auto get_map_service_client =
      std::make_shared<ros::ServiceClient>(
        data->ros_node->serviceClient<nav_msgs::GetMap>(m.second, true));
    if (!get_map_service_client ||
      !get_map_service_client->waitForExistence(ros::Duration(timeout_sec)))
    {
      fferr << "Timed out waiting for GetMap service: " << m.second << "\n";
      return nullptr;
    }

    data->get_map_service_client_map[m.first] =
      std::move(get_map_service_client);
  }

  auto it = data->get_map_service_client_map.find(initial_map_name);
  if (it == data->get_map_service_client_map.end())
  {
    fferr << "Map service for initial map [" << initial_map_name
      << "] not provided.\n";
    return nullptr;
  }

  data->battery_state_sub =
    data->ros_node->subscribe<sensor_msgs::BatteryState>(
      battery_state_topic,
      1,
      [d = data->weak_from_this()](const sensor_msgs::BatteryStateConstPtr& msg)
      {
        const auto data = d.lock();
        if (!data)
          return;
        
        data->battery_state(*msg);
      });  

  auto make_error = [](const std::string& error_message)
  {
    fferr << error_message << "\n";
    return nullptr;
  };

  if (initial_map_name.empty())
    return make_error("initial_map_name cannot be empty.");
  if (map_frame.empty())
    return make_error("map_frame cannot be empty.");
  if (robot_frame.empty())
    return make_error("robot_frame cannot be empty.");
  if (update_frequency == 0)
    return make_error("update_frequency must be larger than 0.");
  
  data->robot_stopped = false;
  data->map_name = initial_map_name;
  data->map_frame = map_frame;
  data->robot_frame = robot_frame;
  data->update_frequency = update_frequency;
  data->timeout_sec = timeout_sec;

  std::shared_ptr<NavStackNode> node(new NavStackNode());
  node->_pimpl->data = std::move(data);
  node->_pimpl->start();
  return node;
}

//==============================================================================
NavStackNode::NavStackNode()
: _pimpl(rmf_utils::make_impl<Implementation>())
{}

//==============================================================================
// std::shared_ptr<NavStackCommandHandle> NavStackNode::command_handle() const
// {
//   if (_pimpl->nav_stack_command_handle)
//     return _pimpl->nav_stack_command_handle;
// 
//   auto command_handle =
//     NavStackCommandHandle::Implementation::make(data->map_frame, data);
//   if (!command_handle)
//   {
//     fferr << "Failed to create a NavStackCommandHandle.\n";
//     return nullptr;
//   }
// 
//   _pimpl->nav_stack_command_handle = std::move(command_handle);
//   return _pimpl->nav_stack_command_handle;
// }

//==============================================================================
std::shared_ptr<NavStackStatusHandle> NavStackNode::status_handle()
{
  if (_pimpl->nav_stack_status_handle)
    return _pimpl->nav_stack_status_handle;

  std::shared_ptr<NavStackStatusHandle> status_handle =
    NavStackStatusHandle::Implementation::make(_pimpl->data);
  if (!status_handle)
  {
    fferr << "Failed to create a NavStackStatusHandle.\n";
    return nullptr;
  }

  _pimpl->nav_stack_status_handle = std::move(status_handle);
  return _pimpl->nav_stack_status_handle;
}

//==============================================================================
} // client
} // free_fleet_ros1
