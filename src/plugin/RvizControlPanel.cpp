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

#include <mutex>
#include <memory>

#include <QLabel>
#include <QString>
#include <QGroupBox>
#include <QTextEdit>
#include <QLineEdit>
#include <QSizePolicy>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>

#include <rviz/panel.h>
#include <rviz/load_resource.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>


#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <free_fleet/messages/NavigationRequest.hpp>
#include <free_fleet_cyclonedds/CycloneDDSMiddleware.hpp>

class RvizControlPanel : public rviz::Panel
{

public:

  RvizControlPanel(QWidget* parent = 0);

private Q_SLOTS:

  void send_nav_goal();

protected:

private:

  void create_robot_group_box();
  void create_nav_group_box();
  void create_debug_group_box();

  QGroupBox* robot_group_box;
  QGroupBox* nav_group_box;
  QGroupBox* debug_group_box;

  QLineEdit* fleet_name_edit;
  QLineEdit* robot_name_edit;
  QTextEdit* nav_goal_edit;
  QLabel* debug_label;

  QPushButton* send_nav_goal_button;

  std::shared_ptr<free_fleet::cyclonedds::CycloneDDSMiddleware> middleware;

  ros::NodeHandle nh;
  ros::Subscriber nav_goal_sub;

  std::mutex nav_goal_mutex;
  geometry_msgs::PoseStamped nav_goal;

  void update_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

  QString nav_goal_to_qstring(const geometry_msgs::PoseStamped& msg) const;

  double get_yaw_from_quat(const geometry_msgs::Quaternion& quat) const;
};

//==============================================================================
RvizControlPanel::RvizControlPanel(QWidget* parent)
: rviz::Panel(parent)
{
  create_robot_group_box();
  create_nav_group_box();
  create_debug_group_box();

  QGridLayout* layout = new QGridLayout;
  layout->addWidget(robot_group_box, 0, 0, 1, 1);
  layout->addWidget(nav_group_box, 1, 0, 6, 1);
  layout->addWidget(debug_group_box, 7, 0, 1, 1);
  setLayout(layout);

  connect(
    send_nav_goal_button,
    &QPushButton::clicked,
    this,
    &RvizControlPanel::send_nav_goal);

  /// create a middleware
  if (!middleware)
  {
    debug_label->setText("CycloneDDS middleware unable to start...");
    return;
  }

  nav_goal_sub =
    nh.subscribe(
      "/move_base_simple/goal",
      2,
      &RvizControlPanel::update_goal,
      this);
}

//==============================================================================
void RvizControlPanel::send_nav_goal()
{
  std::string fleet_name = fleet_name_edit->text().toStdString();
  std::string robot_name = robot_name_edit->text().toStdString();

  using namespace free_fleet::messages;
  Location nav_location;
  NavigationRequest nav_request;

  {
    std::unique_lock<std::mutex> lock(nav_goal_mutex);
    nav_location = {
      static_cast<int32_t>(nav_goal.header.stamp.sec),
      nav_goal.header.stamp.nsec,
      static_cast<float>(nav_goal.pose.position.x),
      static_cast<float>(nav_goal.pose.position.y),
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
void RvizControlPanel::create_robot_group_box()
{
  robot_group_box = new QGroupBox("Robot Selection");
  QGridLayout* layout = new QGridLayout;

  fleet_name_edit = new QLineEdit;
  fleet_name_edit->setPlaceholderText("enter fleet name here");

  robot_name_edit = new QLineEdit;
  robot_name_edit->setPlaceholderText("enter robot name here");

  layout->addWidget(new QLabel("Fleet:"), 0, 0, 1, 1);
  layout->addWidget(fleet_name_edit, 0, 1, 1, 2);
  layout->addWidget(new QLabel("Robot:"), 1, 0, 1, 1);
  layout->addWidget(robot_name_edit, 1, 1, 1, 2);
  robot_group_box->setLayout(layout);
}

//==============================================================================

void RvizControlPanel::create_nav_group_box()
{
  nav_group_box = new QGroupBox("Navigation");
  QGridLayout* layout = new QGridLayout;

  nav_goal_edit = new QTextEdit;
  nav_goal_edit->setReadOnly(true);
  nav_goal_edit->setPlainText(nav_goal_to_qstring(nav_goal));

  send_nav_goal_button = new QPushButton("Send Nav Goal");

  QSizePolicy size_policy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  size_policy.setHorizontalStretch(0);
  size_policy.setVerticalStretch(0);
  size_policy.setHeightForWidth(
    send_nav_goal_button->sizePolicy().hasHeightForWidth());
  send_nav_goal_button->setSizePolicy(size_policy);

  layout->addWidget(nav_goal_edit, 0, 0, 6, 3);
  layout->addWidget(send_nav_goal_button, 0, 3, 6, 1);

  nav_group_box->setLayout(layout);
}

//==============================================================================
void RvizControlPanel::create_debug_group_box()
{
  debug_group_box = new QGroupBox("Debug");
  QHBoxLayout* layout = new QHBoxLayout;

  debug_label = new QLabel("Panel started...");
  layout->addWidget(debug_label);

  debug_group_box->setLayout(layout);
}

//==============================================================================
void RvizControlPanel::update_goal(
    const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::unique_lock<std::mutex> nav_goal_lock(nav_goal_mutex);
  nav_goal = *msg;
  nav_goal_edit->setPlainText(nav_goal_to_qstring(nav_goal));
}

//==============================================================================
QString RvizControlPanel::nav_goal_to_qstring(
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
double RvizControlPanel::get_yaw_from_quat(
  const geometry_msgs::Quaternion& quat) const
{
  tf2::Quaternion tf2_quat;
  tf2::fromMsg(_quat, tf2_quat);
  tf2::Matrix3x3 tf2_mat(tf2_quat);
  
  // ignores pitch and roll, but the api call is so nice though
  double yaw;
  double pitch;
  double roll;
  tf2_mat.getEulerYPR(yaw, pitch, roll);
  return yaw;
}

//==============================================================================
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(RvizControlPanel, rviz::Panel)
