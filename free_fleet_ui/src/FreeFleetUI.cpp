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

#include <iostream>

#include <QMenuBar>
#include <QFileDialog>
#include <QDialog>
#include <QScreen>
#include <QSettings>
#include <QMessageBox>
#include <QGroupBox>
#include <QGridLayout>
#include <QImage>
#include <QImageReader>
#include <QGuiApplication>
#include <QTabWidget>

#include "FreeFleetUI.hpp"

namespace free_fleet
{
namespace viz
{

FreeFleetUI* FreeFleetUI::instance = nullptr;

FreeFleetUI::FreeFleetUI(QWidget* parent) :
  QMainWindow(parent)
{
  instance = this;

  setWindowTitle("Free Fleet User Interface");

  QSettings settings;
  qDebug("settings filename: [%s]", qUtf8Printable(settings.fileName()));

  //===========================================================================
  // Management Column is on the right which will allow tweaking of fleet
  // names, list and selection of robots, display of their current status,
  // also provides high level control of robots in terms of sending mode, 
  // path and destination requests.

  // fleet name group
  fleet_selector.reset(new FleetSelector);
  QGroupBox* fleet_selection_group_box = fleet_selector->make_group_box();

  // robot selection group
  robot_selector.reset(new RobotSelector);
  QGroupBox* robot_selection_group_box = robot_selector->make_group_box();
        
  // robot request group
  robot_requester.reset(new RobotRequester);
  QGroupBox* robot_request_group_box = robot_requester->make_group_box();

  QVBoxLayout* management_column_layout = new QVBoxLayout;
  management_column_layout->addWidget(fleet_selection_group_box);
  management_column_layout->addWidget(robot_selection_group_box);
  management_column_layout->addWidget(robot_request_group_box, 1);

  //===========================================================================
  // Viewer will be displaying the map/laser scan used by all the robots, as
  // well as rendering the locations of each robot on the map.

  scene = new QGraphicsScene(this);
  viewer = new Viewer(this);
  viewer->setScene(scene);

  QVBoxLayout* viewer_layout = new QVBoxLayout;
  viewer_layout->addWidget(viewer);

  //===========================================================================
  // Setting up the overall user interface, putting the management column on
  // the right, before the rest of the window on the left is the viewer.
  // Applying the whole widget as the central widget of the app.

  QHBoxLayout* hbox_layout = new QHBoxLayout;
  hbox_layout->addLayout(management_column_layout, 1);
  hbox_layout->addLayout(viewer_layout, 5);

  QWidget* central_widget = new QWidget();
  central_widget->setMouseTracking(true);
  setMouseTracking(true);

  central_widget->setLayout(hbox_layout);
  // central_widget->setStyleSheet("background-color: #404040");
  setCentralWidget(central_widget);

  //===========================================================================
  // Setting up the top menu bar.

  // FILE MENU
  QMenu* file_menu = menuBar()->addMenu("&File");
  file_menu->addAction(
      "&Open", this, &FreeFleetUI::file_open, QKeySequence(Qt::CTRL + Qt::Key_O));
  file_menu->addAction(
      "&Exit", this, &QWidget::close, QKeySequence(Qt::CTRL + Qt::Key_Q));

  // HELP MENU
  QMenu* help_menu = menuBar()->addMenu("&Help");

  //===========================================================================
  // SET SIZE
  const int width = QGuiApplication::primaryScreen()->availableSize().width();
  const int height = 
      QGuiApplication::primaryScreen()->availableSize().height();
  const int left = 0;
  const int top = 0;

  setGeometry(left, top, width, height);
  // viewer->adjustSize();
}

void FreeFleetUI::file_open()
{
  QFileDialog file_dialog(this, "Open configuration");
  file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.yaml");

  if (file_dialog.exec() != QDialog::Accepted)
    return;
  
  QFileInfo config_file_info(file_dialog.selectedFiles().first());
  if (!config_file_info.exists())
  {
    QMessageBox::critical(
        this,
        "File does not exist",
        "File does not exist. Cannot open file");
    return;
  }

  if (load_config_file(config_file_info))
    create_scene();
}

bool FreeFleetUI::load_config_file(const QFileInfo& config_file_info)
{
  current_map_config = MapConfig::parse_map_config(config_file_info);
  if (!current_map_config)
    return false;

  return true;
}

bool FreeFleetUI::create_scene()
{
  scene->clear();
  // viewer.draw(scene);

  QImageReader image_reader(current_map_config->image);
  image_reader.setAutoTransform(true);
  QImage image = image_reader.read();
  if (image.isNull())
  {
    qWarning("unable to read %s: %s",
        qUtf8Printable(current_map_config->image),
        qUtf8Printable(image_reader.errorString()));
    return false;
  }

  image = image.convertToFormat(QImage::Format_Grayscale8);
  map_pixmap = QPixmap::fromImage(image);
  map_width = map_pixmap.width();
  map_height = map_pixmap.height();

  scene->addPixmap(map_pixmap);

  return true;
}

// void Manager::file_open()
// {
//   QFileDialog file_dialog(this, "Open Config");
//   file_dialog.setFileMode(QFileDialog::ExistingFile);
//   file_dialog.setNameFilter("*.yaml");

//   if (file_dialog.exec() != QDialog::Accepted)
//     return;

//   QFileInfo file_info(file_dialog.selectedFiles().first());
//   if (!file_info.exists()) 
//   {
//     QMessageBox::critical(
//         this,
//         "File does not exist",
//         "File does not exist. Cannot open file.");
//     return;
//   }
//   load_map(file_info.filePath());
// }

FreeFleetUI* FreeFleetUI::get_instance()
{
  return instance;
}

} // namespace viz
} // namespace free_fleet
