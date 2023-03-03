// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>

#include "mjpc/tasks/ur5e/ur5e.h"
#include "mjpc/utilities.h"
#include "mjpc/task.h"

#include <mujoco/mujoco.h>

namespace mjpc {

std::string UR5e::Name() const {
  return GetModelPath("ur5e/task.xml");
}
std::string UR5e::XmlPath() const {
  return std::string();
}
void UR5e::Residual(const mjModel *model, const mjData *data, double *residual) const {

}
void UR5e::Transition(const mjModel *model, mjData *data, mjvScene *scene) {
  Task::Transition(model, data, scene);
}
}  // namespace mjpc
