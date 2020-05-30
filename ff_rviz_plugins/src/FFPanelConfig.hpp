/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef FF_RVIZ_PLUGINS__SRC__FFPANELCONFIG_HPP
#define FF_RVIZ_PLUGINS__SRC__FFPANELCONFIG_HPP

#include <string>

#include <ros/ros.h>

#include <free_fleet/ServerConfig.hpp>

namespace free_fleet {

struct PanelConfig
{
  int state_update_rate = 30;
  std::string fleet_name = "fleet_name";
  std::string rviz_nav_goal_topic = "/move_base_simple/goal";
  std::string panel_state_array_topic = "/ff_panel/robot_state_array";

  int dds_domain = 42;
  std::string dds_robot_state_topic = "robot_state";
  std::string dds_mode_request_topic = "mode_request";
  std::string dds_path_request_topic = "path_request";
  std::string dds_destination_request_topic = "destination_request";

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key, 
      std::string& param_out);

  void get_param_if_available(
      const ros::NodeHandle& node, const std::string& key,
      int& param_out);

  void print_config() const;

  ServerConfig get_server_config() const;

  static PanelConfig make();
};

} // namespace free_fleet

#endif // FF_RVIZ_PLUGINS__SRC__FFPANELCONFIG_HPP
