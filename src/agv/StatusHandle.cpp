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

#include <mutex>
#include <thread>

#include <free_fleet_ros1/agv/StatusHandle.hpp>

namespace free_fleet_ros1 {
namespace agv {

//==============================================================================
class StatusHandle::Implementation
{
public:
  Implementation()
  {}

  Implementation(const Implementation&)
  {
    // Empty copy constructor, only needed during construction of impl_ptr
    // All members will be initialized or assigned during runtime.
  }

  ~Implementation()
  {
    if (_thread.joinable())
      _thread.join();
  }

  ros1::Connections::SharedPtr _connections;

  std::thread _thread;
  mutable std::mutex _mutex;
};

//==============================================================================
StatusHandle::SharedPtr StatusHandle::make(
  ros1::Connections::SharedPtr connections)
{
  StatusHandle::SharedPtr status_handle(new StatusHandle());

  status_handle->_pimpl->_connections = std::move(connections);
  return status_handle;
}

//==============================================================================
StatusHandle::StatusHandle()
: _pimpl(rmf_utils::make_impl<Implementation>(Implementation()))
{}

//==============================================================================
free_fleet::messages::Location StatusHandle::location() const
{
  return free_fleet::messages::Location();
}

//==============================================================================
free_fleet::messages::RobotMode StatusHandle::mode() const
{
  return free_fleet::messages::RobotMode();
}

//==============================================================================
double StatusHandle::battery_percent() const
{
  return 0.0;
}

//==============================================================================
} // namespace agv
} // namespace free_fleet_ros1
