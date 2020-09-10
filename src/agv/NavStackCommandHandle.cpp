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

#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet_ros1/agv/NavStackCommandHandle.hpp>

namespace free_fleet {
namespace agv {

class NavStackCommandHandle::Implementation
{
public:

  using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  using GoalState = actionlib::SimpleClientGoalState;

  Implementation()
  {}

  ~Implementation() = default;

  std::thread _thread;
  std::mutex _mutex;

  std::shared_ptr<MoveBaseClient> _move_base_client;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
  RequestCompleted _path_finished_callback;

};

//==============================================================================

NavStackCommandHandle::SharedPtr NavStackCommandHandle::make(
  std::shared_ptr<ros::NodeHandle> node,
  const std::string& move_base_server_name,
  int timeout)
{
  if (!node)
    return nullptr;

  SharedPtr command_handle(new NavStackCommandHandle());

  ROS_INFO("Waiting for connection with move base action server: %s",
      move_base_server_name.c_str());
  std::shared_ptr<Implementation::MoveBaseClient> move_base_client(
    new Implementation::MoveBaseClient(move_base_server_name, true));
  if (!move_base_client || !move_base_client->waitForServer(ros::Duration(10)))
  {
    ROS_ERROR("Timed out waiting for action server: %s",
        move_base_server_name.c_str());
    return nullptr;
  }
  ROS_INFO("Connected with move base action server: %s",
      move_base_server_name.c_str());

  command_handle->_pimpl->_move_base_client = std::move(move_base_client);
  return command_handle;
}

//==============================================================================

NavStackCommandHandle::NavStackCommandHandle()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================

NavStackCommandHandle::~NavStackCommandHandle()
{}

//==============================================================================

void NavStackCommandHandle::follow_new_path(
  const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
  RequestCompleted path_finished_callback)
{
  
}

//==============================================================================

void NavStackCommandHandle::stop()
{

}

//==============================================================================

void NavStackCommandHandle::resume()
{

}

//==============================================================================

void NavStackCommandHandle::dock(
  const std::string& dock_name,
  RequestCompleted docking_finished_callback)
{

}

//==============================================================================

} // namespace agv
} // namespace free_fleet
