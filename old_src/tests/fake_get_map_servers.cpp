/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <nav_msgs/GetMap.h>

bool get_map_call(
  nav_msgs::GetMap::Request& request,
  nav_msgs::GetMap::Response& response)
{
  ROS_INFO("Getting map");
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_get_map_servers");
  ros::NodeHandle n;

  auto service_0 = n.advertiseService("fake_get_map_server_0", get_map_call);
  auto service_1 = n.advertiseService("fake_get_map_server_1", get_map_call);
  auto service_2 = n.advertiseService("fake_get_map_server_2", get_map_call);
  ROS_INFO("Fake get_map_servers running.");
  ros::spin();
  return 0;
}
