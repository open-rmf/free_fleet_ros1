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

#include <QMenuBar>

#include "Manager.hpp"

namespace free_fleet
{
namespace viz
{

Manager* Manager::instance = nullptr;

Manager::Manager(QWidget* parent) :
  QMainWindow(parent)
{
  instance = this;

  setWindowTitle("Free Fleet Manager");

  QVBoxLayout* manager_column_layout = new QVBoxLayout;

  scene = new QGraphicsScene(this);
  viewer = new Viewer(this);
  QVBoxLayout* viewer_layout = new QVBoxLayout;
  viewer_layout->addWidget(viewer);

  QHBoxLayout* hbox_layout = new QHBoxLayout;
  hbox_layout->addLayout(manager_column_layout);
  hbox_layout->addLayout(viewer_layout, 1);

  QWidget* central_widget = new QWidget();
  central_widget->setMouseTracking(true);
  setMouseTracking(true);
  central_widget->setLayout(hbox_layout);
  central_widget->setStyleSheet("background-color: #404040");
  setCentralWidget(central_widget);

  // FILE MENU
  QMenu* file_menu = menuBar()->addMenu("&File");

  // HELP MENU
  QMenu* help_menu = menuBar()->addMenu("&Help");
}

Manager* Manager::get_instance()
{
  return instance;
}

} // namespace viz
} // namespace free_fleet
