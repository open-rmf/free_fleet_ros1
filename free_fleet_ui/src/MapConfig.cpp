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

#include <yaml-cpp/yaml.h>

#include "MapConfig.hpp"

namespace free_fleet
{
namespace viz
{

MapConfig::SharedPtr MapConfig::parse_map_config(
    const std::string& config_path)
{
  YAML::Node yaml;
  try
  {
    yaml = YAML::LoadFile(config_path.c_str());
  }
  catch (const std::exception& e)
  {
    printf("couldn't parse %s: %s", config_path.c_str(), e.what());
    return nullptr;
  }

  if (!yaml["image"] || !yaml["resolution"] || !yaml["origin"])
  {
    printf("couldn't parse %s: configuration must have image, resolution and "
        "origin fields", config_path.c_str());
    return nullptr;
  }

  auto new_config = std::make_shared<MapConfig>();
  return new_config;
}

} // namespace viz
} // namespace free_fleet
