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

#ifndef SRC__FREE_FLEET_ROS1__CLIENT__INTERNAL_NAVSTACKSTATUSHANDLE_HPP
#define SRC__FREE_FLEET_ROS1__CLIENT__INTERNAL_NAVSTACKSTATUSHANDLE_HPP

#include <memory>
#include <string>
#include <thread>
#include <optional>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <free_fleet_ros1/client/NavStackStatusHandle.hpp>

#include "NavStackData.hpp"

namespace free_fleet_ros1 {
namespace client {

//==============================================================================
class NavStackStatusHandle::Implementation
{
public:

  static std::shared_ptr<NavStackStatusHandle> make(
    std::shared_ptr<NavStackData> nav_stack_data);
  
  Implementation()
  {
    if (thread.joinable())
      thread.join();
  }

  Implementation(const Implementation&)
  {
    // Empty copy constructor, only needed during construction of impl_ptr
    // All members will be initialized or assigned during runtime.
  }

  void update_transforms();

  void update_mode();

  void update_thread_fn();

  bool initialized() const;

  void start();

  std::thread thread;

  std::shared_ptr<NavStackData> nav_stack_data;

  std::optional<geometry_msgs::TransformStamped> previous_transform =
    std::nullopt;

  std::optional<geometry_msgs::TransformStamped> current_transform =
    std::nullopt;

  std::unique_ptr<ros::Rate> update_rate;
};


//==============================================================================
double quat_to_yaw(const geometry_msgs::Quaternion& quat);

//==============================================================================
bool is_transform_close(
  const geometry_msgs::TransformStamped& first,
  const geometry_msgs::TransformStamped& second);

//==============================================================================
} // namespace client
} // namespace free_fleet_ros1

#endif // SRC__FREE_FLEET_ROS1__CLIENT__INTERNAL_NAVSTACKSTATUSHANDLE_HPP
