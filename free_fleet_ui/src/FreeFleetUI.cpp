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
#include <QMessageBox>

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

  //===========================================================================
  // Management Column is on the right which will allow tweaking of fleet
  // names, list and selection of robots, display of their current status,
  // also provides high level control of robots in terms of sending mode, 
  // path and destination requests.

  QVBoxLayout* management_column_layout = new QVBoxLayout;

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
  hbox_layout->addLayout(management_column_layout);
  hbox_layout->addLayout(viewer_layout, 1);

  QWidget* central_widget = new QWidget();
  central_widget->setMouseTracking(true);
  setMouseTracking(true);
  central_widget->setLayout(hbox_layout);
  central_widget->setStyleSheet("background-color: #404040");
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
}

void FreeFleetUI::file_open()
{
  QFileDialog file_dialog(this, "Open configuration");
  file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.yaml");

  if (file_dialog.exec() != QDialog::Accepted)
    return;
  
  QFileInfo file_info(file_dialog.selectedFiles().first());
  if (!file_info.exists())
  {
    QMessageBox::critical(
        this,
        "File does not exist",
        "File does not exist. Cannot open file");
    return;
  }
  load_file(file_info);
}

bool FreeFleetUI::load_file(const QFileInfo& file_info)
{
  const std::string dir_path_std_string =
      file_info.dir().absolutePath().toStdString();
  current_map_config.reset(new MapConfig)



  const std::string file_path_std_string =
      file_info.filePath().toStdString();

  std::cout << "dir: " << dir_path_std_string << std::endl;
  std::cout << "file: " << file_path_std_string << std::endl;
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
