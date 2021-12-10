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

#include <vector>
#include <catch_ros/catch.hpp>
#include <ros/init.h>

#include <free_fleet_ros1/client/NavStackNode.hpp>

TEST_CASE("Verify that the node will timeout")
{
  int argc = 1;
	std::vector<char*> argv{strdup("dummy")};
	ros::init(argc, argv.data(), "dummy");
  free(argv[0]);

  auto n = free_fleet_ros1::client::NavStackNode::make(
    "test_node_name",
    "test_move_base_server_name",
    "test_set_map_server_name",
    {},
    "test_battery_state_topic",
    "test_initial_map_name",
    "test_map_frame",
    "test_robot_frame",
    10,
    1);
  CHECK(nullptr == n);
}
