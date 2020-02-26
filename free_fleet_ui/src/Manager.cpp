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
#include <QFileDialog>
#include <QDialog>
#include <QFileInfo>

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
  viewer->setScene(scene);

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

bool Manager::load_config(const QString& filename)
{
  const std::string filename_std_string = filename.toStdString();
  try
  {

  }
  catch (const std::exception &e)
  {
    qWarning("couldn't parse %s: %s",
        qUtf8Printable(filename),
        e.what());
    return false;
  }

  return true;
}

void Manager::open_config()
{
  QFileDialog file_dialog(this, "Open Config");
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
        "File does not exist. Cannot open file.");
    return;
  }
  load_map(file_info.filePath());
}

Manager* Manager::get_instance()
{
  return instance;
}

} // namespace viz
} // namespace free_fleet
