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

#include <free_fleet_ros1/agv/NavStackCommandHandle.hpp>

namespace free_fleet {
namespace agv {

class NavStackCommandHandle::Implementation
{

};

//==============================================================================

NavStackCommandHandle::SharedPtr NavStackCommandHandle::make(
  const std::string& move_base_server_name)
{
  
}

//==============================================================================

NavStackCommandHandle::NavStackCommandHandle()
{}

//==============================================================================

NavStackCommandHandle::~NavStackCommandHandle()
{}

//==============================================================================

} // namespace agv
} // namespace free_fleet
