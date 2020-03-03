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

#ifndef FREE_FLEET_UI__SRC__VIEWER_HPP
#define FREE_FLEET_UI__SRC__VIEWER_HPP

#include <cstdio>

#include <QWheelEvent>
#include <QMouseEvent>
#include <QGraphicsView>
#include <QGraphicsScene>

#include "MapConfig.hpp"

namespace free_fleet
{
namespace viz
{

class Viewer : public QGraphicsView
{

  Q_OBJECT

public:

  Viewer(QWidget* parent = nullptr);

  bool create_scene(const MapConfig::SharedPtr& map_config);

protected:

  void wheelEvent(QWheelEvent* event) final;

  void mouseMoveEvent(QMouseEvent* event) final;

  void mousePressEvent(QMouseEvent* event) final;

  void mouseReleaseEvent(QMouseEvent* event) final;

  bool is_panning;

  int pan_start_x;

  int pan_start_y;

private:

  QGraphicsScene* scene;

  QPixmap map_pixmap;

  int map_width = 0;

  int map_height = 0;

  MapConfig::SharedPtr map_config;

  QPointF mouse_to_real_pos(const QPoint& mouse_pos) const;

};

} // namespace viz
} // namespace free_fleet

#endif // FREE_FLEET_UI__SRC__VIEWER_HPP
