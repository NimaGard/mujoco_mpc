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

#include "mjpc/tasks/peg_in_hole/peg_in_hole.h"
#include "mjpc/utilities.h"
#include "mjpc/task.h"

#include <mujoco/mujoco.h>

namespace mjpc {

std::string PegInHole::Name() const {
  return "Peg in Hole";
}
std::string PegInHole::XmlPath() const {
  return GetModelPath("peg_in_hole/task.xml");
}
void PegInHole::Residual(const mjModel *model, const mjData *data, double *residual) const {

}
void PegInHole::Transition(const mjModel *model, mjData *data) {
  Task::Transition(model, data);
}

void PegInHole::Reset(const mjModel* model) {

}

}  // namespace mjpc
