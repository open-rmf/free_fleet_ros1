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

#include <ros/ros.h>
#include <ff_rviz_plugins_msgs/RobotState.h>
#include <ff_rviz_plugins_msgs/RobotStateArray.h>

#include <free_fleet/Server.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/RobotState.hpp>

#include "utilities.hpp"
#include "FFPanelConfig.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ff_panel_server_node");
  ros::NodeHandle n;

  auto panel_config = free_fleet::PanelConfig::make();

  ros::Rate loop_rate(panel_config.state_update_rate);
  ros::Publisher robot_state_pub = 
      n.advertise<ff_rviz_plugins_msgs::RobotStateArray>(
          panel_config.panel_state_array_topic, 10);

  auto server_ptr = free_fleet::Server::make(panel_config.get_server_config());
  if (!server_ptr)
  {
    ROS_ERROR("server_ptr is a nullptr");
    return 1;
  }

  while (ros::ok())
  {
    std::vector<free_fleet::messages::RobotState> new_robot_states;
    if (server_ptr->read_robot_states(new_robot_states))
    {
      ff_rviz_plugins_msgs::RobotStateArray new_array_msg;
      free_fleet::convert(new_robot_states, new_array_msg);
      robot_state_pub.publish(new_array_msg);
    }
    loop_rate.sleep();
  }

  return 0;
}
