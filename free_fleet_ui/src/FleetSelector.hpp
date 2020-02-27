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

#ifndef FREE_FLEET_UI__SRC__FLEETSELECTOR_HPP
#define FREE_FLEET_UI__SRC__FLEETSELECTOR_HPP

#include <memory>

#include <QGroupBox>

namespace free_fleet
{
namespace viz
{

class FleetSelector
{

public:

  using UniquePtr = std::unique_ptr<FleetSelector>;

  FleetSelector();

  QGroupBox* make_group_box(QWidget* parent = nullptr);

private:

};

} // namespace viz
} // namespace free_fleet

#endif // FREE_FLEET_UI__SRC__FLEETSELECTOR_HPP
