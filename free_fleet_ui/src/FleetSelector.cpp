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

#include <cstdio>

#include <QObject>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include "FleetSelector.hpp"

namespace free_fleet
{
namespace viz
{

FleetSelector::FleetSelector()
{}

QGroupBox* FleetSelector::make_group_box()
{
  fleet_name_editor = new QLineEdit;

  QPushButton* refresh_fleet_name_button = new QPushButton("refresh");
  // refresh_fleet_name_button->setStyleSheet(
  //     "QGroupBox {background-color: #e0e0e0;}");

  QHBoxLayout* h_layout_1 = new QHBoxLayout;
  h_layout_1->addWidget(new QLabel("Name:"), 4);
  h_layout_1->addWidget(fleet_name_editor, 12);
  h_layout_1->addWidget(new QWidget, 1);
  h_layout_1->addWidget(refresh_fleet_name_button, 4);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(h_layout_1);

  QGroupBox* group_box = new QGroupBox("Fleet");
  // group_box->setStyleSheet("QGroupBox {background-color: #e0e0e0;}");
  group_box->setLayout(layout);

  QGroupBox::connect(
      refresh_fleet_name_button, 
      &QPushButton::clicked,
      [=](){this->refresh_fleet_name();});

  return group_box;
}

QString FleetSelector::get_fleet_name()
{
  ReadLock fleet_name_lock(fleet_name_mutex);
  return fleet_name;
}

void FleetSelector::set_robot_num(int n)
{
  // robot_num_display->setText((std::to_string(n)).c_str());
}

void FleetSelector::refresh_fleet_name()
{
  printf("refresh button was clicked\n");
}

} // namespace viz
} // namespace free_fleet
