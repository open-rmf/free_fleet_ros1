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

#ifndef FF_RVIZ_PLUGINS__SRC__FFPANEL_HPP
#define FF_RVIZ_PLUGINS__SRC__FFPANEL_HPP

#include <mutex>
#include <unordered_map>

#include <QLabel>
#include <QString>
#include <QLineEdit>
#include <QTextEdit>
#include <QGroupBox>
#include <QComboBox>
#include <QPushButton>

#include <rviz/panel.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <ff_rviz_plugins_msgs/RobotStateArray.h>

#include <free_fleet/Server.hpp>

namespace free_fleet {

class FFPanel : public rviz::Panel
{

Q_OBJECT

public:

  FFPanel(QWidget* parent = 0);

public Q_SLOTS:

  void update_robot_name_selector();
  void send_nav_goal();

private:

  QGroupBox* create_robot_group_box();
  QGroupBox* create_nav_group_box();
  QGroupBox* create_debug_group_box();

  void create_layout();
  void create_connections();

  QLabel* _fleet_name;
  QComboBox* _robot_name_selector;

  QTextEdit* _nav_goal_edit;
  QPushButton* _send_nav_goal_button;

  QLabel* _debug_label;

  // TODO: figure out why we are unable to read incoming states.
  // For now, we will use a server relay, using static topics and Path messages.
  Server::SharedPtr _free_fleet_server;

  ros::NodeHandle _nh;
  ros::Subscriber _nav_goal_sub;
  ros::Subscriber _state_array_relay_sub;

  std::mutex _nav_goal_mutex;
  geometry_msgs::PoseStamped _nav_goal;

  std::mutex _robot_states_mutex;
  std::unordered_map<std::string, ff_rviz_plugins_msgs::RobotState> 
      _robot_states;

  void update_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void update_states(const ff_rviz_plugins_msgs::RobotStateArray::ConstPtr& msg);

  QString nav_goal_to_qstring(const geometry_msgs::PoseStamped& msg) const;
};

} // namespace free_fleet

#endif // FF_RVIZ_PLUGINS__SRC__FFPANEL_HPP
