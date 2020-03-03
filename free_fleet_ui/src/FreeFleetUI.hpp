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

#ifndef FREE_FLEET_UI__SRC__FREEFLEETUI_HPP
#define FREE_FLEET_UI__SRC__FREEFLEETUI_HPP

#include <memory>

#include <QMainWindow>
#include <QGraphicsScene>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileInfo>
#include <QPixmap>

#include "Viewer.hpp"
#include "MapConfig.hpp"
#include "FleetSelector.hpp"
#include "RobotSelector.hpp"
#include "RobotRequester.hpp"

namespace free_fleet
{
namespace viz
{

class FreeFleetUI : public QMainWindow
{
  
  Q_OBJECT

public:

  FreeFleetUI(QWidget* parent = nullptr);

private:

  /// There will only be one instance
  static FreeFleetUI* instance;

  static FreeFleetUI* get_instance();

  Viewer* viewer;

  MapConfig::SharedPtr current_map_config;

  void file_open();

  FleetSelector::UniquePtr fleet_selector;

  RobotSelector::UniquePtr robot_selector;

  RobotRequester::UniquePtr robot_requester;

};

} // namespace viz
} // namespace free_fleet

#endif // FREE_FLEET_UI__SRC__FREEFLEETUI_HPP
