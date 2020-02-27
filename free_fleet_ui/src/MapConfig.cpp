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
#include <string>
#include <yaml-cpp/yaml.h>

#include <QDir>
#include <QString>

#include "MapConfig.hpp"

namespace free_fleet
{
namespace viz
{

MapConfig::MapConfig()
{}

void MapConfig::print_config()
{
  printf("Map Config:\n");
  printf("  image path: %s\n", image.toStdString().c_str());
  printf("  resolution: %.2f\n", resolution);
  printf("  origin    : %.2f  %.2f  %.2f\n", origin[0], origin[1], origin[2]);
}

MapConfig::SharedPtr MapConfig::parse_map_config(
    const QFileInfo& config_file_info)
{
  const QDir config_dir = config_file_info.dir();
  const std::string config_path_std_string = 
      config_file_info.filePath().toStdString();

  YAML::Node yaml;
  try
  {
    yaml = YAML::LoadFile(config_path_std_string.c_str());
  }
  catch (const std::exception& e)
  {
    printf("couldn't parse %s: %s", config_path_std_string.c_str(), e.what());
    return nullptr;
  }

  if (!yaml["image"] || !yaml["resolution"] || !yaml["origin"])
  {
    printf("couldn't parse %s: configuration must have image, resolution and "
        "origin fields", config_path_std_string.c_str());
    return nullptr;
  }

  auto new_config = std::make_shared<MapConfig>();
 
  std::string image_relative_path_std_string = yaml["image"].as<std::string>();
  const QString image_relative_path = 
      QString::fromStdString(image_relative_path_std_string);
  if (!config_dir.exists(image_relative_path))
  {
    printf(
        "image file does not exist: %s", 
        image_relative_path_std_string.c_str());
    return nullptr;
  }

  new_config->image = config_dir.absoluteFilePath(image_relative_path);
  new_config->resolution = yaml["resolution"].as<double>();

  YAML::Node origin_node = yaml["origin"];
  for (int i = 0; i < 3; ++i)
    new_config->origin[i] = origin_node[i].as<double>();

  new_config->print_config();
  return new_config;
}

} // namespace viz
} // namespace free_fleet
