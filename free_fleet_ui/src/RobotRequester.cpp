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

#include <QTabWidget>
#include <QVBoxLayout>

#include "RobotRequester.hpp"

namespace free_fleet
{
namespace viz
{

RobotRequester::RobotRequester()
{}

QGroupBox* RobotRequester::make_group_box(QWidget* parent)
{
  QGroupBox* group_box = new QGroupBox("Robot Requests", parent);
  // group_box->setStyleSheet("QGroupBox {background-color: #e0e0e0;}");

  QTabWidget* robot_request_tabs = new QTabWidget;
  robot_request_tabs->addTab(new QWidget, "Mode");
  robot_request_tabs->addTab(new QWidget, "Destination");
  robot_request_tabs->addTab(new QWidget, "Path");
  // robot_request_tabs->setStyleSheet("QTabBar::tab { color: white; }");

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(robot_request_tabs);
  group_box->setLayout(layout);

  return group_box;
}

} // namespace viz 
} // namespace free_fleet
