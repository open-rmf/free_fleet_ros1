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
#include <QListView>
#include <QStringList>
#include <QPushButton>
#include <QRadioButton>
#include <QStringListModel>

#include <rviz/panel.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <ff_rviz_plugins_msgs/RobotStateArray.h>

#include <free_fleet/Server.hpp>

namespace free_fleet {

class FFPanel : public rviz::Panel
{

Q_OBJECT

public:
  
  using ReadLock = std::unique_lock<std::mutex>;
  using WriteLock = std::unique_lock<std::mutex>;

  using Marker = visualization_msgs::Marker;
  using PoseStamped = geometry_msgs::PoseStamped;
  using MarkerArray = visualization_msgs::MarkerArray;
  using RobotMode = ff_rviz_plugins_msgs::RobotMode;
  using RobotState = ff_rviz_plugins_msgs::RobotState;
  using RobotStateArray = ff_rviz_plugins_msgs::RobotStateArray;

  struct RobotFields
  {
    RobotState state;
    MarkerArray markers;
  };

  FFPanel(QWidget* parent = 0);

public Q_SLOTS:

  void update_robot_name_selector();
  void updated_robot_name();
  void clear_goals();
  void delete_waypoint();
  void send_goals();
  void send_mode_request();

private:

  QGroupBox* create_robot_group_box();
  QGroupBox* create_state_group_box();
  QGroupBox* create_nav_group_box();
  QGroupBox* create_mode_group_box();
  QGroupBox* create_debug_group_box();

  void create_layout();
  void create_connections();

  QLabel* _fleet_name;
  QComboBox* _robot_name_selector;

  std::vector<QLabel*> _state_labels;

  QListView* _nav_goal_list_view;
  QStringListModel* _nav_goal_list_model;
  QStringList _nav_goal_str_list;

  QPushButton* _clear_goals_button;
  QPushButton* _delete_waypoint_button;
  QPushButton* _send_goals_button;

  QRadioButton* _pause_radio_button;
  QRadioButton* _resume_radio_button;

  QPushButton* _send_mode_request_button;

  QLabel* _debug_label;

  // TODO: figure out why we are unable to read incoming states.
  // For now, we will use a server relay, using static topics and Path messages.
  Server::SharedPtr _free_fleet_server;

  ros::NodeHandle _nh;
  ros::Subscriber _rviz_nav_goal_sub;
  ros::Subscriber _state_array_relay_sub;

  rviz_visual_tools::RvizVisualToolsPtr _visual_tools;

  std::mutex _robots_mutex;
  std::unordered_map<std::string, RobotFields> _robots;

  void rviz_nav_goal_callback(const PoseStamped::ConstPtr& msg);

  void update_states(const RobotStateArray::ConstPtr& msg);

  void display_goal_list();

  void display_markers();

  void display_state(const RobotState& state);

  void clear_markers() const;

  QString marker_to_qstring(const Marker& marker) const;

  QString robot_mode_qstring(uint32_t mode) const;
};

} // namespace free_fleet

#endif // FF_RVIZ_PLUGINS__SRC__FFPANEL_HPP
