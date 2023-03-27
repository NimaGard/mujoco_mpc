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

#include "mjpc/tasks/manipulation/peg_in_hole.h"
#include "mjpc/utilities.h"
#include "mjpc/task.h"
#include "mjpc/tasks/manipulation/common.h"

#include <string>
#include <absl/container/flat_hash_map.h>
#include <absl/random/random.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

namespace mjpc {
namespace manipulation {
std::string PegInHole::Name() const {
  return "Panda Peg in Hole";
}
std::string PegInHole::XmlPath() const {
  return GetModelPath("manipulation/task_peg_in_hole.xml");
}
void PegInHole::Residual(const mjModel *model, const mjData *data, double *residual) const {
  int counter = 0;

  // grab the object
  double* hand = SensorByName(model, data, "hand");
  double* object = SensorByName(model, data, "box");
  mju_sub3(residual + counter, hand, object);

  counter += 3;
 // bring
  double* box1 = SensorByName(model, data, "box1");
  double* target1 = SensorByName(model, data, "target1");
  mju_sub3(residual + counter, box1, target1);
  counter += 3;
  double* box2 = SensorByName(model, data, "box2");
  double* target2 = SensorByName(model, data, "target2");
  mju_sub3(residual + counter, box2, target2);
  counter += 3;


  // sensor dim sanity check
  CheckSensorDim(model, counter);

}
void PegInHole::Transition(const mjModel *model, mjData *data) {
  double residuals[100];
  double terms[10];
  Residual(model, data, residuals);
  CostTerms(terms, residuals);
  double bring_dist = (mju_norm3(residuals + 3) + mju_norm3(residuals + 6)) / 2;

  // reset:
  if (data->time > 0 && bring_dist < .015) {
    // box:
    absl::BitGen gen_;
    data->qpos[0] = absl::Uniform<double>(gen_, -.5, .5);
    data->qpos[1] = absl::Uniform<double>(gen_, -.5, .5);
    data->qpos[2] = .05;

    // target:
    data->mocap_pos[0] = absl::Uniform<double>(gen_, -.5, .5);
    data->mocap_pos[1] = absl::Uniform<double>(gen_, -.5, .5);
    data->mocap_pos[2] = absl::Uniform<double>(gen_, .03, 1);
    data->mocap_quat[0] = absl::Uniform<double>(gen_, -1, 1);
    data->mocap_quat[1] = absl::Uniform<double>(gen_, -1, 1);
    data->mocap_quat[2] = absl::Uniform<double>(gen_, -1, 1);
    data->mocap_quat[3] = absl::Uniform<double>(gen_, -1, 1);
    mju_normalize4(data->mocap_quat);
  }
}

void PegInHole::Reset(const mjModel *model) {
  Task::Reset(model);
  model_vals_ = ModelValues::FromModel(model);
}
}  // namespace manipulation
}  // namespace mjpc
