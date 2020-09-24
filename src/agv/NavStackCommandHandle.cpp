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
#include <deque>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Quaternion.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet_ros1/agv/NavStackCommandHandle.hpp>

namespace free_fleet {
namespace agv {

//==============================================================================
class NavStackCommandHandle::Implementation
{
public:

  struct Goal
  {
    rmf_traffic::agv::Plan::Waypoint waypoint;
    move_base_msgs::MoveBaseGoal goal;
    bool sent = false;
  };

  using MoveBaseClient =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
  using GoalState = actionlib::SimpleClientGoalState;

  Implementation()
  {}

  Implementation(const Implementation& other)
  {
    // Empty copy constructor, only needed during construction of impl_ptr
    // All members will be initialized or assigned during runtime.
  }

  ~Implementation()
  {
    if (_thread.joinable())
      _thread.join();
  }

  void _navigation_thread_fn()
  {
    while (_node->ok())
    {
      // check goal state
      // pop and send if needed
      _navigation_handle_rate->sleep();
    }
  }

  void _start()
  {
    ROS_INFO("Starting navigation thread.");
    _thread =
      std::thread(std::bind(&Implementation::_navigation_thread_fn, this));
  }

  geometry_msgs::Quaternion _get_quat_from_yaw(double yaw) const
  {
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0.0, 0.0, yaw);
    quat_tf.normalize();
    geometry_msgs::Quaternion quat = tf2::toMsg(quat_tf);
    return quat;
  }

  move_base_msgs::MoveBaseGoal _waypoint_to_move_base_goal(
      const rmf_traffic::agv::Plan::Waypoint& waypoint) const
  {
    const Eigen::Vector3d& pos = waypoint.position();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = _map_frame;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = pos[0];
    goal.target_pose.pose.position.y = pos[1];
    goal.target_pose.pose.position.z = 0.0; // TODO: handle Z height with level
    goal.target_pose.pose.orientation = _get_quat_from_yaw(pos[2]);
    return goal;
  }

  std::thread _thread;
  mutable std::mutex _mutex;

  std::shared_ptr<ros::NodeHandle> _node;
  std::shared_ptr<MoveBaseClient> _move_base_client;
  std::unique_ptr<ros::Rate> _navigation_handle_rate;

  std::string _map_frame;

  std::deque<Goal> _goal_path;
  RequestCompleted _path_finished_callback;

  RequestCompleted _docking_finished_callback;
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

  command_handle->_pimpl->_node = std::move(node);
  command_handle->_pimpl->_move_base_client = std::move(move_base_client);
  command_handle->_pimpl->_navigation_handle_rate.reset(
    new ros::Rate(1));
  command_handle->_pimpl->_start();
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
  ROS_INFO("Got a new path goal of length: %lu", waypoints.size());
  
  {
    std::lock_guard<std::mutex> lock(_pimpl->_mutex);
    _pimpl->_goal_path.clear();
    for (std::size_t i = 0; i < waypoints.size(); ++i)
    {
      _pimpl->_goal_path.push_back(
        Implementation::Goal {
          waypoints[i],
          _pimpl->_waypoint_to_move_base_goal(waypoints[i]),
          false});
    }
  }

  _pimpl->_move_base_client->cancelAllGoals();
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  if (!_pimpl->_goal_path.empty())
  {
    const Implementation::Goal& next_goal = _pimpl->_goal_path.front();
    if (!next_goal.sent)
    {
      const Eigen::Vector3d& pos = next_goal.waypoint.position();
      ROS_INFO("sending next goal: %.3f %.3f %.3f", pos[0], pos[1], pos[2]);
      _pimpl->_move_base_client->sendGoal(next_goal.goal);
      _pimpl->_goal_path.front().sent = true;
    }
  }
}

//==============================================================================
void NavStackCommandHandle::stop()
{
  _pimpl->_move_base_client->cancelAllGoals();
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  if (!_pimpl->_goal_path.empty())
    _pimpl->_goal_path[0].sent = false;
}

//==============================================================================
void NavStackCommandHandle::resume()
{
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  if (!_pimpl->_goal_path.empty())
  {
    const Implementation::Goal& next_goal = _pimpl->_goal_path.front();
    if (!next_goal.sent)
    {
      const Eigen::Vector3d& pos = next_goal.waypoint.position();
      ROS_INFO("sending next goal: %.3f %.3f %.3f", pos[0], pos[1], pos[2]);
      _pimpl->_move_base_client->sendGoal(next_goal.goal);
      _pimpl->_goal_path.front().sent = true;
    }
  }
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
