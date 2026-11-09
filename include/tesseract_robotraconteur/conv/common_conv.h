/**
 * @file common_conv.h
 *
 * @author John Wason, PhD
 *
 * @copyright Copyright 2023 Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * @par
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TESSERACT_ROBOTRACONTEUR_COMMON_CONV_H
#define TESSERACT_ROBOTRACONTEUR_COMMON_CONV_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/joint_state.h>

#include "robotraconteur_generated.h"

namespace rr_common = experimental::tesseract_robotics::common;

namespace tesseract_robotraconteur
{
namespace conv
{
rr_common::ManipulatorInfoPtr ManipulatorInfoToRR(const tesseract::common::ManipulatorInfo& manip_info);

tesseract::common::ManipulatorInfo ManipulatorInfoFromRR(const rr_common::ManipulatorInfoPtr& manip_info);

rr_common::JointStatePtr JointStateToRR(const tesseract::common::JointState& joint_state);

tesseract::common::JointState JointStateFromRR(const rr_common::JointStatePtr& joint_state);

rr_common::JointTrajectoryPtr JointTrajectoryToRR(const tesseract::common::JointTrajectory& joint_trajectory);

tesseract::common::JointTrajectory JointTrajectoryFromRR(const rr_common::JointTrajectoryPtr& joint_trajectory);

} // namespace conv
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_COMMON_CONV_H
