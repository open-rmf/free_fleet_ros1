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

#include <std_srvs/Empty.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/SetMap.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <free_fleet/messages/Location.hpp>

#include <free_fleet_ros1/agv/NavStackCommandHandle.hpp>

namespace free_fleet_ros1 {
namespace agv {

//==============================================================================
class NavStackCommandHandle::Implementation
{
public:

  struct Goal
  {
    free_fleet::messages::Waypoint waypoint;
    move_base_msgs::MoveBaseGoal goal;
    bool sent = false;
  };

  Implementation()
  {}

  Implementation(const Implementation&)
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
      _navigation_handle_rate->sleep();
      std::lock_guard<std::mutex> lock(_mutex);
      
      if (_connections->stopped() || _goal_path.empty())
        continue;

      // Goals must have been updated since last handling, execute them now
      if (!_goal_path.front().sent)
      {
        const Goal& next_goal = _goal_path.front();
        const auto& loc = next_goal.waypoint.location;
        ROS_INFO("Sending next goal: %.3f %.3f %.3f", loc.x, loc.y, loc.yaw);

        _connections->move_base_client()->sendGoal(next_goal.goal);
        _goal_path.front().sent = true;
        continue;
      }

      // check goal state
      // pop and send if needed
      using GoalState = actionlib::SimpleClientGoalState;
      GoalState current_goal_state =
        _connections->move_base_client()->getState();
      if (current_goal_state == GoalState::SUCCEEDED)
      {
        ROS_INFO("Current goal state: SUCCEEEDED.");

        // TODO (AA): Implement it in a way that would ignore time difference
        // between the clients and the server.

        // By some stroke of good fortune, we may have arrived at our goal
        // earlier than we were scheduled to reach it. If that is the case,
        // we need to wait here until it's time to proceed.
        const ros::Time waypoint_end_time =
          _goal_path.front().goal.target_pose.header.stamp;
        if (ros::Time::now() < waypoint_end_time)
        {
          ros::Duration wait_time_remaining =
            waypoint_end_time - ros::Time::now();
          ROS_INFO(
            "We reached our goal early! Waiting %.1f more seconds",
            wait_time_remaining.toSec());
        }
        else
        {
          _goal_path.pop_front();

          std::size_t next_path_index = _connections->next_path_index() + 1;
          _connections->next_path_index(next_path_index);

          if (_goal_path.empty() && _path_finished_callback)
            _path_finished_callback();
        }
      }
      else if (current_goal_state == GoalState::ACTIVE)
      {
        // This means the robot is still moving towards the current goal.
        continue;
      }
      else
      {
        ROS_INFO(
          "Current goal state: %s", current_goal_state.toString().c_str());

        // TODO (AA): figure out what states actually require intervention from
        // handlers, and possibly return a request_error on mode.
        // ROS_INFO("Intervention might be required.");
      }
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

  move_base_msgs::MoveBaseGoal _location_to_move_base_goal(
    const free_fleet::messages::Location& location) const
  {
    // TODO(AA): handle Z height with level
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = _map_frame;
    goal.target_pose.header.stamp.sec = location.sec;
    goal.target_pose.header.stamp.nsec = location.nanosec;
    goal.target_pose.pose.position.x = location.x;
    goal.target_pose.pose.position.y = location.y;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation = _get_quat_from_yaw(location.yaw);
    return goal;
  }

  geometry_msgs::PoseWithCovarianceStamped _location_to_pose_with_cov(
    const free_fleet::messages::Location& location) const
  {
    // TODO(AA): handle Z height with level
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = _map_frame;
    msg.header.stamp.sec = location.sec;
    msg.header.stamp.nsec = location.nanosec;
    msg.pose.pose.position.x = location.x;
    msg.pose.pose.position.y = location.y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = _get_quat_from_yaw(location.yaw);
    msg.pose.covariance = {
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787};
    return msg;
  }

  std::thread _thread;
  mutable std::mutex _mutex;

  ros1::Connections::SharedPtr _connections;
  std::shared_ptr<ros::NodeHandle> _node;
  std::unique_ptr<ros::Rate> _navigation_handle_rate;

  std::string _map_frame;

  std::deque<Goal> _goal_path;
  RequestCompleted _path_finished_callback;

  RequestCompleted _docking_finished_callback;
};

//==============================================================================
NavStackCommandHandle::SharedPtr NavStackCommandHandle::make(
  ros1::Connections::SharedPtr connections,
  const std::string& map_frame)
{
  if (!connections)
    return nullptr;

  SharedPtr command_handle(new NavStackCommandHandle());
  command_handle->_pimpl->_connections = std::move(connections);
  command_handle->_pimpl->_node = command_handle->_pimpl->_connections->node();
  command_handle->_pimpl->_navigation_handle_rate.reset(new ros::Rate(1));
  command_handle->_pimpl->_map_frame = map_frame;
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
void NavStackCommandHandle::relocalize(
  const free_fleet::messages::Location& location,
  RequestCompleted relocalization_finished_callback)
{
  auto get_map_client =
    _pimpl->_connections->get_map_client(location.level_name);
  if (!get_map_client)
  {
    ROS_WARN("Received relocalization request to an unsupported level/map: %s",
      location.level_name.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  _pimpl->_goal_path.clear();
  _pimpl->_connections->move_base_client()->cancelAllGoals();
  _pimpl->_connections->path({});

  nav_msgs::GetMap::Request get_req;
  nav_msgs::GetMap::Response get_resp;
  if (!get_map_client->call(get_req, get_resp))
  {
    ROS_WARN(
      "Unable to get map [%s] from service: [%s]",
      location.level_name.c_str(),
      get_map_client->getService().c_str());
    return;
  }

  nav_msgs::SetMap::Request set_req;
  set_req.map = get_resp.map;
  set_req.initial_pose = _pimpl->_location_to_pose_with_cov(location);
  nav_msgs::SetMap::Response set_resp;
  
  auto set_map_client = _pimpl->_connections->set_map_service_client();
  if (!set_map_client->call(set_req, set_resp) || !set_resp.success)
  {
    ROS_WARN(
      "Unable to set map and location to service: [%s]",
      set_map_client->getService().c_str());
    return;
  }

  _pimpl->_connections->map_name(location.level_name);
  ROS_INFO("Relocalized to: %.3f, %.3f, Yaw: %.3f, Level/Map: %s",
    location.x, location.y, location.yaw, location.level_name.c_str());
}

//==============================================================================
void NavStackCommandHandle::follow_new_path(
    const std::vector<free_fleet::messages::Waypoint>& waypoints,
    RequestCompleted path_finished_callback)
{
  ROS_INFO("Got a new path goal of length: %lu", waypoints.size());

  // TODO(AA): Check that all waypoints are in the current map/level
  
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  _pimpl->_goal_path.clear();
  for (std::size_t i = 0; i < waypoints.size(); ++i)
  {
    _pimpl->_goal_path.push_back(
      Implementation::Goal {
        waypoints[i],
        _pimpl->_location_to_move_base_goal(waypoints[i].location),
        false});
  }
  _pimpl->_path_finished_callback = std::move(path_finished_callback);
  _pimpl->_connections->move_base_client()->cancelAllGoals();
  _pimpl->_connections->path(waypoints);
}

//==============================================================================
void NavStackCommandHandle::stop()
{
  if (_pimpl->_connections->stopped())
    return;

  _pimpl->_connections->stopped(true);
  _pimpl->_connections->move_base_client()->cancelAllGoals();
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  if (!_pimpl->_goal_path.empty())
    _pimpl->_goal_path[0].sent = false;
}

//==============================================================================
void NavStackCommandHandle::resume()
{
  if (!_pimpl->_connections->stopped())
    return;

  _pimpl->_connections->stopped(false);
  std::lock_guard<std::mutex> lock(_pimpl->_mutex);
  if (!_pimpl->_goal_path.empty())
  {
    const Implementation::Goal& next_goal = _pimpl->_goal_path.front();
    if (!next_goal.sent)
    {
      const auto& loc = next_goal.waypoint.location;
      ROS_INFO("sending next goal: %.3f %.3f %.3f", loc.x, loc.y, loc.yaw);
      _pimpl->_connections->move_base_client()->sendGoal(next_goal.goal);
      _pimpl->_goal_path.front().sent = true;
    }
  }
}

//==============================================================================
void NavStackCommandHandle::dock(const std::string&, RequestCompleted)
{
  // TODO (AA): To support any docking systems in the ROS 1 navigation stack
}

//==============================================================================
} // namespace agv
} // namespace free_fleet_ros1
