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

#include <string>

#include <QSizePolicy>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

#include <free_fleet/ServerConfig.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/RobotMode.hpp>
#include <free_fleet/messages/RobotState.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>

#include <visualization_msgs/Marker.h>

#include "FFPanel.hpp"
#include "FFPanelConfig.hpp"
#include "utilities.hpp"

namespace free_fleet {

//==============================================================================

FFPanel::FFPanel(QWidget* parent)
: rviz::Panel(parent)
{
  create_layout();
  create_connections();

  auto panel_config = PanelConfig::make();

  _fleet_name->setText(QString::fromStdString(panel_config.fleet_name));
  
  _free_fleet_server = Server::make(panel_config.get_server_config());
  if (!_free_fleet_server)
  {
    _debug_label->setText("Free Fleet server unable to start...");
    return;
  }

  _rviz_nav_goal_sub = _nh.subscribe(
      panel_config.rviz_nav_goal_topic, 2, 
      &FFPanel::rviz_nav_goal_callback, this);
  _state_array_relay_sub = _nh.subscribe(
      panel_config.panel_state_array_topic, 2, 
      &FFPanel::update_states, this);

  _visual_tools.reset(
      new rviz_visual_tools::RvizVisualTools(
          "map", panel_config.navigation_markers_topic));
}

//==============================================================================

void FFPanel::update_robot_name_selector()
{
  _robot_name_selector->blockSignals(true);
  _robot_name_selector->clear();

  ReadLock robots_lock(_robots_mutex);
  for (const auto it : _robots)
    _robot_name_selector->addItem(QString(it.first.c_str()));
  robots_lock.unlock();

  _robot_name_selector->model()->sort(0);
  _robot_name_selector->blockSignals(false);
}

//==============================================================================

void FFPanel::updated_robot_name()
{
  const std::string selected_robot =
      _robot_name_selector->currentText().toStdString();

  ReadLock robots_lock(_robots_mutex);
  auto it = _robots.find(selected_robot);
  if (it == _robots.end())
    return;

  display_state(it->second.state);
  robots_lock.unlock();
  
  display_goal_list();
  display_markers();
}

//==============================================================================

void FFPanel::clear_goals()
{
  const std::string selected_robot =
      _robot_name_selector->currentText().toStdString();

  ReadLock robots_lock(_robots_mutex);
  auto it = _robots.find(selected_robot);
  if (it == _robots.end())
    return;

  it->second.markers.markers = {};
  robots_lock.unlock();

  display_goal_list();
  display_markers();
}

//==============================================================================

void FFPanel::delete_waypoint()
{
  const std::string selected_robot =
      _robot_name_selector->currentText().toStdString();

  ReadLock robots_lock(_robots_mutex);
  auto it = _robots.find(selected_robot);
  if (it == _robots.end() || it->second.markers.markers.empty())
    return;

  // Pop both the arrow and text marker
  it->second.markers.markers.pop_back();
  it->second.markers.markers.pop_back();
  robots_lock.unlock();

  display_goal_list();
  display_markers();
}

//==============================================================================

void FFPanel::send_goals()
{
  const std::string selected_robot =
      _robot_name_selector->currentText().toStdString();
  if (selected_robot.empty())
    return;

  messages::PathRequest path_request;
  path_request.fleet_name = _fleet_name->text().toStdString();
  path_request.robot_name = selected_robot;
  path_request.path.clear();
  path_request.task_id = generate_random_task_id(20);

  ReadLock robots_lock(_robots_mutex);
  const auto it = _robots.find(selected_robot);
  for (const auto& m : it->second.markers.markers)
  {
    ros::Time time_now = ros::Time::now();
    path_request.path.push_back({
      static_cast<int32_t>(time_now.sec),
      time_now.nsec,
      static_cast<float>(m.pose.position.x),
      static_cast<float>(m.pose.position.y),
      static_cast<float>(get_yaw_from_quat(m.pose.orientation)),
      ""
    });
  }
  robots_lock.unlock();

  std::string debug_str = _free_fleet_server->send_path_request(path_request) ?
      "Sent navigation request..." :
      "Failed to send navigation request...";
  _debug_label->setText(QString::fromStdString(debug_str));
}

//==============================================================================

void FFPanel::send_mode_request()
{
  const std::string selected_robot =
      _robot_name_selector->currentText().toStdString();
  if (selected_robot.empty())
    return;

  messages::ModeRequest mode_request;
  mode_request.fleet_name = _fleet_name->text().toStdString();
  mode_request.robot_name = selected_robot;
  if (_pause_radio_button->isChecked())
    mode_request.mode.mode = messages::RobotMode::MODE_PAUSED;
  else if (_resume_radio_button->isChecked())
    mode_request.mode.mode = messages::RobotMode::MODE_MOVING;
  mode_request.task_id = generate_random_task_id(20);

  std::string debug_str = _free_fleet_server->send_mode_request(mode_request) ?
      "Sent mode request..." :
      "Failed to send mode request...";
  _debug_label->setText(QString::fromStdString(debug_str));
}

//==============================================================================

QGroupBox* FFPanel::create_robot_group_box()
{
  QLabel* fleet_name_label = new QLabel("Fleet:");
  fleet_name_label->setStyleSheet("font: italic;");

  QLabel* robot_name_label = new QLabel("Robot:");
  robot_name_label->setStyleSheet("font: italic;");

  _fleet_name = new QLabel("");

  _robot_name_selector = new QComboBox;

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(fleet_name_label, 0, 0, 1, 1);
  layout->addWidget(_fleet_name, 0, 1, 1, 3);
  layout->addWidget(robot_name_label, 1, 0, 1, 1);
  layout->addWidget(_robot_name_selector, 1, 1, 1, 3);

  QGroupBox* group_box = new QGroupBox("Selection");
  group_box->setLayout(layout);
  return group_box;
}

//==============================================================================

QGroupBox* FFPanel::create_state_group_box()
{
  std::vector<QLabel*> field_labels = {
    new QLabel("model"),
    new QLabel("task id"),
    new QLabel("mode"),
    new QLabel("battery (%)"),
    new QLabel("location"),
    new QLabel("time (sec)"),
    new QLabel("x (m)"),
    new QLabel("y (m)"),
    new QLabel("yaw (rad)"),
    new QLabel("level")
  };
  for (auto& f : field_labels)
    f->setStyleSheet(
        "border-width: 1px;"
        "border-style: solid;"
        "border-color: transparent darkgray darkgray transparent;"
        "font: italic;");

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(field_labels[0], 0, 0, 1, 2);
  layout->addWidget(field_labels[1], 1, 0, 1, 2);
  layout->addWidget(field_labels[2], 2, 0, 1, 2);
  layout->addWidget(field_labels[3], 3, 0, 1, 2);
  layout->addWidget(field_labels[4], 4, 0, 5, 1);
  layout->addWidget(field_labels[5], 4, 1, 1, 1);
  layout->addWidget(field_labels[6], 5, 1, 1, 1);
  layout->addWidget(field_labels[7], 6, 1, 1, 1);
  layout->addWidget(field_labels[8], 7, 1, 1, 1);
  layout->addWidget(field_labels[9], 8, 1, 1, 1);

  _state_labels.clear();
  for (size_t i = 0; i < 9; ++i)
  {
    QLabel* state_label = new QLabel("");
    state_label->setStyleSheet(
        "border-width: 1px;"
        "border-style: solid;"
        "border-color: transparent transparent darkgray transparent;");
    _state_labels.push_back(state_label);
    layout->addWidget(state_label, i, 2, 1, 2);
  }

  QGroupBox* group_box = new QGroupBox("State");
  group_box->setLayout(layout);
  return group_box;
}

//==============================================================================

QGroupBox* FFPanel::create_nav_group_box()
{
  _nav_goal_str_list = QStringList();
  _nav_goal_list_model = new QStringListModel;
  _nav_goal_list_view = new QListView;
  _nav_goal_list_view->setModel(_nav_goal_list_model);
  _nav_goal_list_view->setToolTip(QString("x(m), y(m), yaw(rad)"));

  _clear_goals_button = new QPushButton("&Clear Goal");
  _delete_waypoint_button = new QPushButton("&Delete Waypoint");
  _send_goals_button = new QPushButton("&Send Goal");

  QSizePolicy size_policy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  size_policy.setHorizontalStretch(0);
  size_policy.setVerticalStretch(0);
  size_policy.setHeightForWidth(
      _clear_goals_button->sizePolicy().hasHeightForWidth());
  _clear_goals_button->setSizePolicy(size_policy);
  _delete_waypoint_button->setSizePolicy(size_policy);
  _send_goals_button->setSizePolicy(size_policy);

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(_nav_goal_list_view, 0, 0, 6, 3);
  layout->addWidget(_clear_goals_button, 0, 3, 2, 1);
  layout->addWidget(_delete_waypoint_button, 2, 3, 2, 1);
  layout->addWidget(_send_goals_button, 4, 3, 2, 1);

  QGroupBox* group_box = new QGroupBox("Navigation");
  group_box->setLayout(layout);
  return group_box;
}

//==============================================================================

QGroupBox* FFPanel::create_mode_group_box()
{
  _pause_radio_button = new QRadioButton("Pause");
  _pause_radio_button->setChecked(true);
  _resume_radio_button = new QRadioButton("Resume");

  _send_mode_request_button = new QPushButton("Send &Mode Request");

  QSizePolicy size_policy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  size_policy.setHorizontalStretch(0);
  size_policy.setVerticalStretch(0);
  size_policy.setHeightForWidth(
      _send_mode_request_button->sizePolicy().hasHeightForWidth());
  _send_mode_request_button->setSizePolicy(size_policy);

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(_pause_radio_button, 0, 0, 1, 1);
  layout->addWidget(_resume_radio_button, 1, 0, 1, 1);
  layout->addWidget(_send_mode_request_button, 0, 1, 2, 1);

  QGroupBox* group_box = new QGroupBox("Mode");
  group_box->setLayout(layout);
  return group_box;
}

//==============================================================================

QGroupBox* FFPanel::create_debug_group_box()
{
  _debug_label = new QLabel("Panel started...");

  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget(_debug_label);

  QGroupBox* group_box = new QGroupBox("Debug");
  group_box->setLayout(layout);
  return group_box;
}

//==============================================================================

void FFPanel::create_layout()
{
  QGroupBox* robot_gb = create_robot_group_box();
  QGroupBox* state_gb = create_state_group_box();
  QGroupBox* navigation_gb = create_nav_group_box();
  QGroupBox* mode_gb = create_mode_group_box();
  QGroupBox* debug_gb = create_debug_group_box();

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(robot_gb, 0, 0, 1, 1);
  layout->addWidget(state_gb, 1, 0, 6, 1);
  layout->addWidget(navigation_gb, 7, 0, 6, 1);
  layout->addWidget(mode_gb, 13, 0, 2, 1);
  layout->addWidget(debug_gb, 15, 0, 1, 1);
  setLayout(layout);
  setStyleSheet(
      "QGroupBox {"
      "  font: bold;"
      "  border: 1px solid silver;"
      "  border-radius: 6px;"
      "  margin-top: 6px;"
      "  padding-top: 10px;"
      "}"
      "QGroupBox::title {"
      "  subcontrol-origin: margin;"
      "  left: 7px;"
      "  padding: 0px 5px 0px 5px;"
      "}"
  );
}

//==============================================================================

void FFPanel::create_connections()
{
  connect(this, SIGNAL(configChanged()), this,
      SLOT(update_robot_name_selector()));
  connect(_robot_name_selector, SIGNAL(currentTextChanged(const QString &)),
      this, SLOT(updated_robot_name()));

  connect(_clear_goals_button, &QPushButton::clicked, this,
      &FFPanel::clear_goals);
  connect(_delete_waypoint_button, &QPushButton::clicked, this,
      &FFPanel::delete_waypoint);
  connect(_send_goals_button, &QPushButton::clicked, this,
      &FFPanel::send_goals);
  connect(_send_mode_request_button, &QPushButton::clicked, this,
      &FFPanel::send_mode_request);
}

//==============================================================================

void FFPanel::rviz_nav_goal_callback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  const std::string selected_robot =
      _robot_name_selector->currentText().toStdString();
  if (selected_robot.empty())
    return;

  WriteLock robots_lock(_robots_mutex);
  const auto it = _robots.find(selected_robot);
  if (it == _robots.end())
    return;

  int current_id = static_cast<int>(it->second.markers.markers.size() / 2);

  visualization_msgs::Marker arrow_marker;
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.header.frame_id = "map";
  arrow_marker.ns = selected_robot + "/arrow";
  arrow_marker.id = current_id;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.action = visualization_msgs::Marker::MODIFY;
  arrow_marker.pose = msg->pose;
  arrow_marker.scale.x = 0.7;
  arrow_marker.scale.y = 0.1;
  arrow_marker.scale.z = 0.1;
  arrow_marker.color.r = 1.0;
  arrow_marker.color.g = 0.0;
  arrow_marker.color.b = 0.0;
  arrow_marker.color.a = 0.6;

  visualization_msgs::Marker text_marker;
  text_marker.header.stamp = ros::Time::now();
  text_marker.header.frame_id = "map";
  text_marker.ns = selected_robot + "/text";
  text_marker.id = current_id;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::MODIFY;
  text_marker.pose = msg->pose;
  text_marker.pose.position.x -= 0.2;
  text_marker.scale.z = 0.3;
  text_marker.color.r = 1.0;
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.0;
  text_marker.color.a = 1.0;
  text_marker.text = std::to_string(current_id);

  it->second.markers.markers.push_back(arrow_marker);
  it->second.markers.markers.push_back(text_marker);
  robots_lock.unlock();

  display_goal_list();
  display_markers();
}

//==============================================================================

void FFPanel::update_states(
    const ff_rviz_plugins_msgs::RobotStateArray::ConstPtr& msg)
{
  const std::string selected_robot =
      _robot_name_selector->currentText().toStdString();

  std::unique_lock<std::mutex> robots_lock(_robots_mutex);
  bool new_robot_found = false;
  for (const auto& rs : msg->states)
  {
    std::string robot_name = rs.name;
    if (_robots.find(robot_name) == _robots.end())
    {
      new_robot_found = true;
      _robots[rs.name] = RobotFields {
        rs,
        MarkerArray()
      };
    }
    else
    {
      _robots[rs.name].state = rs;
    }

    if (robot_name == selected_robot)
      display_state(rs);
  }
  robots_lock.unlock();

  std::string debug_label_str = new_robot_found ?
      "New robot found, refreshing..." :
      "Currently handling " + std::to_string(_robots.size()) + " robots...";
  _debug_label->setText(QString::fromStdString(debug_label_str));

  if (new_robot_found)
    Q_EMIT configChanged();
}

//==============================================================================

void FFPanel::display_goal_list()
{
  const std::string selected_robot =
      _robot_name_selector->currentText().toStdString();

  ReadLock robots_lock(_robots_mutex);
  const auto it = _robots.find(selected_robot);
  if (it == _robots.end())
    return;
  
  _nav_goal_str_list.clear();
  for (size_t i = 0; i < it->second.markers.markers.size(); i+=2)
    _nav_goal_str_list.append(marker_to_qstring(it->second.markers.markers[i]));
  robots_lock.unlock();
  
  _nav_goal_list_model->setStringList(_nav_goal_str_list);
  _nav_goal_list_view->scrollToBottom();
}

//==============================================================================

void FFPanel::display_markers()
{
  clear_markers();
  
  const std::string selected_robot =
      _robot_name_selector->currentText().toStdString();
  
  ReadLock robots_lock(_robots_mutex);
  const auto it = _robots.find(selected_robot);
  if (it == _robots.end())
    return;

  for (auto& m : it->second.markers.markers)
    m.header.stamp = ros::Time::now();

  _visual_tools->publishMarkers(it->second.markers);
}

//==============================================================================

void FFPanel::display_state(const RobotState& state)
{
  _state_labels[0]->setText(
      QString::fromStdString(state.model));
  _state_labels[1]->setText(
      QString::fromStdString(state.task_id));
  _state_labels[2]->setText(
      robot_mode_qstring(state.mode.mode));
  _state_labels[3]->setText(
      QString::fromStdString(std::to_string(state.battery_percent)));
  _state_labels[4]->setText(
      QString::fromStdString(std::to_string(state.location.t.sec)));
  _state_labels[5]->setText(
      QString::fromStdString(std::to_string(state.location.x)));
  _state_labels[6]->setText(
      QString::fromStdString(std::to_string(state.location.y)));
  _state_labels[7]->setText(
      QString::fromStdString(std::to_string(state.location.yaw)));
  _state_labels[8]->setText(
      QString::fromStdString(state.location.level_name));
}

//==============================================================================

void FFPanel::clear_markers() const
{
  _visual_tools->deleteAllMarkers();
  _visual_tools->trigger();
}

//==============================================================================

QString FFPanel::marker_to_qstring(const Marker& marker) const
{
  auto to_string_precision = [](double val, int precision) 
  {
    std::ostringstream out;
    out.precision(precision);
    out << std::fixed << val;
    return out.str();
  };

  std::ostringstream ss;
  ss << "x: " << to_string_precision(marker.pose.position.x, 2) 
      << ", y: " << to_string_precision(marker.pose.position.y, 2) 
      << ", yaw: " 
      << to_string_precision(get_yaw_from_quat(marker.pose.orientation), 2);

  return QString::fromStdString(ss.str());
}

//==============================================================================

QString FFPanel::robot_mode_qstring(uint32_t mode) const
{
  switch (mode) {
    case RobotMode::MODE_IDLE:
      return QString("Idle");
    case RobotMode::MODE_CHARGING:
      return QString("Charging");
    case RobotMode::MODE_MOVING:
      return QString("Moving");
    case RobotMode::MODE_PAUSED:
      return QString("Paused");
    case RobotMode::MODE_WAITING:
      return QString("Waiting");
    case RobotMode::MODE_EMERGENCY:
      return QString("Emergency");
    case RobotMode::MODE_GOING_HOME:
      return QString("Going Home");
    case RobotMode::MODE_DOCKING:
      return QString("Docking");
    case RobotMode::MODE_ADAPTER_ERROR:
      return QString("Adapter Error");
    default:
      return QString("Undefined");
  }
}

//==============================================================================

} // namespace free_fleet

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(free_fleet::FFPanel, rviz::Panel)
