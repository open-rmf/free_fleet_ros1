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

#include <QImage>
#include <QPixmap>
#include <QScrollBar>
#include <QImageReader>

#include "Viewer.hpp"

namespace free_fleet
{
namespace viz
{

Viewer::Viewer(QWidget* parent)
: QGraphicsView(parent),
  is_panning(false),
  pan_start_x(0),
  pan_start_y(0)
{
  setMouseTracking(true);
  viewport()->setMouseTracking(true);
  setTransformationAnchor(QGraphicsView::NoAnchor);

  scene = new QGraphicsScene(this);
  setScene(scene);
}

bool Viewer::create_scene(const MapConfig::SharedPtr& _map_config)
{
  if (!_map_config)
    return false;

  map_config = _map_config;

  scene->clear();
  // viewer.draw(scene);

  QImageReader image_reader(_map_config->image);
  image_reader.setAutoTransform(true);
  QImage image = image_reader.read();
  if (image.isNull())
  {
    qWarning("unable to read %s: %s",
        qUtf8Printable(_map_config->image),
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

void Viewer::wheelEvent(QWheelEvent* event)
{
  // calculate the map position before we scale things
  const QPointF p_start = mapToScene(event->pos());
  
  // scale things
  if (event->delta() > 0)
    scale(1.1, 1.1);
  else
    scale(0.9, 0.9);

  // calculate the mouse map position now that we've scaled
  const QPointF p_end = mapToScene(event->pos());

  // translate the map back so hopefully the mouse stays in the same spot
  const QPointF diff = p_end - p_start;
  translate(diff.x(), diff.y());
}

void Viewer::mouseMoveEvent(QMouseEvent* event)
{
  if (is_panning)
  {
    const int dx = event->x() - pan_start_x;
    const int dy = event->y() - pan_start_y;
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - dx);
    verticalScrollBar()->setValue(verticalScrollBar()->value() - dy);
    pan_start_x = event->x();
    pan_start_y = event->y();
    event->accept();
    return;
  }
  event->ignore();
}

void Viewer::mousePressEvent(QMouseEvent* event)
{
  if (event->button() == Qt::MiddleButton)
  {
    is_panning = true;
    pan_start_x = event->x();
    pan_start_y = event->y();
    event->accept();
    return;
  }
  event->ignore();
}

void Viewer::mouseReleaseEvent(QMouseEvent* event)
{
  if (event->button() == Qt::MiddleButton)
  {
    is_panning = false;
    event->accept();
    return;
  }
  else if (event->button() == Qt::LeftButton)
  {
    const QPointF real_pos = mouse_to_real_pos(event->pos());
    printf("clicked ,x: %.2f, y: %.2f\n", real_pos.x(), real_pos.y());
    event->accept();
    return;
  }
  event->ignore();
}

QPointF Viewer::mouse_to_real_pos(const QPoint& mouse_pos) const
{
  const QPointF map_pos = mapToScene(mouse_pos);
  const QPointF scaled_real_pos = map_pos * map_config->resolution;
  const QPointF real_pos_with_offset(
      scaled_real_pos.x() + map_config->origin[0],
      -(scaled_real_pos.y() + map_config->origin[1]));
      
  // TODO: check offsets again, there seem to be some difference between
  // RVIZ clicked_point and real_pos_with_offset.
  return real_pos_with_offset;
}

} // namespace viz
} // namespace free_fleet
