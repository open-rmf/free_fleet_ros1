/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "NavStackData.hpp"

namespace free_fleet_ros1 {
namespace client {

//==============================================================================
auto NavStackData::next_path_index() const
-> std::optional<std::size_t>
{
  ReadLock lock(_next_path_index_mutex);
  return _next_path_index;
}

//==============================================================================
void NavStackData::next_path_index(std::optional<std::size_t> index)
{
  WriteLock lock(_next_path_index_mutex);
  _next_path_index = index;
}

//==============================================================================
auto NavStackData::battery_state() const
-> std::optional<sensor_msgs::BatteryState>
{
  ReadLock lock(_battery_state_mutex);
  return _battery_state;
}

//==============================================================================
void NavStackData::battery_state(const sensor_msgs::BatteryState& state)
{
  WriteLock lock(_battery_state_mutex);
  _battery_state = state;
}

//==============================================================================
auto NavStackData::mode() const
-> std::optional<free_fleet::messages::RobotMode>
{
  ReadLock lock(_mode_mutex);
  return _mode;
}

//==============================================================================
void NavStackData::mode(const free_fleet::messages::RobotMode& mode)
{
  WriteLock lock(_mode_mutex);
  _mode = mode;
}

//==============================================================================
auto NavStackData::location() const
-> std::optional<free_fleet::messages::Location>
{
  ReadLock lock(_loc_mutex);
  return _location;
}

//==============================================================================
void NavStackData::location(const free_fleet::messages::Location& location)
{
  WriteLock lock(_loc_mutex);
  _location = location;
}

//==============================================================================
} // namespace client
} // namespace free_fleet_ros1
