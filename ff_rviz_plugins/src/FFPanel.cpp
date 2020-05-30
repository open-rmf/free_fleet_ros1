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
}

//==============================================================================

void FFPanel::send_nav_goal()
{
  std::string fleet_name = _fleet_name->text().toStdString();
  std::string robot_name = _robot_name_selector->currentText().toStdString();

  messages::Location nav_goal_location;
  messages::DestinationRequest destination_request;

  {
    std::unique_lock<std::mutex> nav_goal_lock(_nav_goal_mutex);
    nav_goal_location = {
      static_cast<int32_t>(_nav_goal.header.stamp.sec),
      _nav_goal.header.stamp.nsec,
      static_cast<float>(_nav_goal.pose.position.x),
      static_cast<float>(_nav_goal.pose.position.y),
      static_cast<float>(get_yaw_from_quat(_nav_goal.pose.orientation)),
      ""
    };
    destination_request = {
      std::move(fleet_name),
      std::move(robot_name),
      std::move(nav_goal_location),
      generate_random_task_id(20)
    };
  }

  if (!_free_fleet_server->send_destination_request(destination_request))
  {
    std::string debug_str = "Failed to send navigation request...";
    _debug_label->setText(QString::fromStdString(debug_str));
    return;
  }
}

//==============================================================================

void FFPanel::update_robot_name_selector()
{
  _robot_name_selector->blockSignals(true);
  _robot_name_selector->clear();

  std::unique_lock<std::mutex> robot_states_lock(_robot_states_mutex);
  for (auto it : _robot_states)
  {
    _robot_name_selector->addItem(QString(it.first.c_str()));
  }
  _robot_name_selector->model()->sort(0);

  _robot_name_selector->blockSignals(false);
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

QGroupBox* FFPanel::create_nav_group_box()
{
  _nav_goal_edit = new QTextEdit;
  _nav_goal_edit->setReadOnly(true);
  _nav_goal_edit->setPlainText(nav_goal_to_qstring(_nav_goal));

  _delete_waypoint_button = new QPushButton("&Delete Waypoint");
  _send_goal_button = new QPushButton("&Send Goal");

  QSizePolicy size_policy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  size_policy.setHorizontalStretch(0);
  size_policy.setVerticalStretch(0);
  size_policy.setHeightForWidth(
      _send_nav_goal_button->sizePolicy().hasHeightForWidth());
  _send_nav_goal_button->setSizePolicy(size_policy);

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(_nav_goal_edit, 0, 0, 6, 3);
  layout->addWidget(_delete_waypoint_button, 0, 3, 3, 1);
  layout->addWidget(_send_goal_button, 3, 3, 3, 1);

  QGroupBox* group_box = new QGroupBox("Navigation");
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
  QGroupBox* navigation_gb = create_nav_group_box();
  QGroupBox* debug_gb = create_debug_group_box();

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(robot_gb, 0, 0, 1, 1);
  layout->addWidget(navigation_gb, 1, 0, 6, 1);
  layout->addWidget(debug_gb, 7, 0, 1, 1);
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
  connect(_send_nav_goal_button, &QPushButton::clicked, this,
      &FFPanel::send_nav_goal);
}

//==============================================================================

void FFPanel::rviz_nav_goal_callback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::unique_lock<std::mutex> nav_goal_lock(_nav_goal_mutex);
  _nav_goals.push_back(*msg);
  nav_goal_lock.unlock();
  display_goals();
  _nav_goal_edit->setPlainText(nav_goal_to_qstring(_nav_goal));
}

//==============================================================================

void FFPanel::update_states(
    const ff_rviz_plugins_msgs::RobotStateArray::ConstPtr& msg)
{
  std::unique_lock<std::mutex> robot_states_lock(_robot_states_mutex);
  bool new_robot_found = false;
  for (const auto& rs : msg->states)
  {
    std::string robot_name = rs.name;
    if (_robot_states.find(robot_name) == _robot_states.end())
      new_robot_found = true;
    _robot_states[rs.name] = rs;
  }

  robot_states_lock.unlock();

  if (new_robot_found)
  {
    std::string debug_label_str = "New robot found, refreshing...";
    _debug_label->setText(QString::fromStdString(debug_label_str));
    Q_EMIT configChanged();
  }
  else
  {
    std::string debug_label_str = 
        "Currently handling " 
        + std::to_string(_robot_states.size()) 
        + " robots...";
    _debug_label->setText(QString::fromStdString(debug_label_str));
  }
}

//==============================================================================

void FFPanel::display_goals()
{
}

//==============================================================================

QString FFPanel::nav_goal_to_qstring(
    const geometry_msgs::PoseStamped& msg) const
{
  std::ostringstream ss;
  ss <<
      "Position:" <<
      "\n    x: " << std::to_string(msg.pose.position.x) <<
      "\n    y: " << std::to_string(msg.pose.position.y) <<
      "\n    z: " << std::to_string(msg.pose.position.z) <<
      "\nOrientation:" <<
      "\n    x: " << std::to_string(msg.pose.orientation.x) <<
      "\n    y: " << std::to_string(msg.pose.orientation.y) <<
      "\n    z: " << std::to_string(msg.pose.orientation.z) <<
      "\n    w: " << std::to_string(msg.pose.orientation.w) << std::endl;
  return QString::fromStdString(ss.str());
}

//==============================================================================

} // namespace free_fleet

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(free_fleet::FFPanel, rviz::Panel)
