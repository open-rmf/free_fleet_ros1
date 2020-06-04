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

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <ff_rviz_plugins_msgs/RobotStateArray.h>

#include <free_fleet/Server.hpp>

namespace free_fleet {

class FFPanel : public rviz::Panel
{

Q_OBJECT

public:

  using Pose = geometry_msgs::Pose;
  using Marker = visualization_msgs::Marker;
  using PoseStamped = geometry_msgs::PoseStamped;
  using MarkerArray = visualization_msgs::MarkerArray;
  using RobotStateArray = ff_rviz_plugins_msgs::RobotStateArray;

  struct NavMarkers
  {
    MarkerArray arrow_markers;
    MarkerArray text_markers;
  };

  FFPanel(QWidget* parent = 0);

public Q_SLOTS:

  void update_robot_name_selector();
  void update_nav_goal();
  void clear_nav_goal();
  void delete_waypoint();
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

  QListView* _nav_goal_list_view;
  QStringListModel* _nav_goal_list_model;
  QStringList _nav_goal_str_list;

  QPushButton* _clear_goal_button;
  QPushButton* _delete_waypoint_button;
  QPushButton* _send_goal_button;

  QLabel* _debug_label;

  // TODO: figure out why we are unable to read incoming states.
  // For now, we will use a server relay, using static topics and Path messages.
  Server::SharedPtr _free_fleet_server;

  ros::NodeHandle _nh;
  ros::Subscriber _rviz_nav_goal_sub;
  ros::Subscriber _state_array_relay_sub;
  ros::Publisher _nav_goal_markers_pub;

  std::mutex _robot_states_mutex;
  std::unordered_map<std::string, ff_rviz_plugins_msgs::RobotState> 
      _robot_states;

  std::mutex _nav_markers_mutex;
  std::unordered_map<std::string, NavMarkers> _nav_markers_map;
  std::string _current_nav_markers_owner;

  void rviz_nav_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void clear_nav_markers(const NavMarkers& nav_markers) const;

  void set_nav_markers(const NavMarkers& nav_markers) const;

  void set_goal_list(const NavMarkers& nav_markers);

  void update_states(const ff_rviz_plugins_msgs::RobotStateArray::ConstPtr& msg); 

  QString marker_to_qstring(const Marker& marker);
};

} // namespace free_fleet

#endif // FF_RVIZ_PLUGINS__SRC__FFPANEL_HPP
