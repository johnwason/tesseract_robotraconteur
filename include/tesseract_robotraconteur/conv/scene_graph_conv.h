/**
 * @file scene_graph_conv.h
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

#ifndef TESSERACT_ROBOTRACONTEUR_SCENE_GRAPH_CONV_H
#define TESSERACT_ROBOTRACONTEUR_SCENE_GRAPH_CONV_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "robotraconteur_generated.h"

#include <tesseract/environment/commands.h>

#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/graph.h>

#include "geometry_conv.h"

namespace rr_sg = experimental::tesseract_robotics::scene_graph;
namespace rr_geom = com::robotraconteur::geometry;

namespace tesseract_robotraconteur
{
namespace environment_conv
{
// Visual
rr_sg::VisualPtr VisualToRR(const tesseract::scene_graph::Visual& visual);

rr_sg::VisualPtr VisualToRR(const tesseract::scene_graph::Visual::ConstPtr& visual);

tesseract::scene_graph::Visual::Ptr VisualFromRR(const rr_sg::VisualPtr& visual);

// LinkMaterial

rr_shapes::MaterialPtr LinkMaterialToRR(const tesseract::scene_graph::Material& link_material,
                                        const std::shared_ptr<const tesseract::geometry::Geometry>& geom);

rr_shapes::MaterialPtr LinkMaterialToRR(const tesseract::scene_graph::Material::ConstPtr& link_material,
                                        const std::shared_ptr<const tesseract::geometry::Geometry>& geom);

tesseract::scene_graph::Material::Ptr LinkMaterialFromRR(const rr_shapes::MaterialPtr& link_material);

// Collision

rr_sg::CollisionPtr CollisionToRR(const tesseract::scene_graph::Collision& collision);

rr_sg::CollisionPtr CollisionToRR(const tesseract::scene_graph::Collision::ConstPtr& collision);

tesseract::scene_graph::Collision::Ptr CollisionFromRR(const rr_sg::CollisionPtr& collision);

// Joint Dynamics

rr_sg::JointDynamicsPtr JointDynamicsToRR(const tesseract::scene_graph::JointDynamics& joint_dynamics);

rr_sg::JointDynamicsPtr JointDynamicsToRR(const tesseract::scene_graph::JointDynamics::ConstPtr joint_dynamics);

tesseract::scene_graph::JointDynamics::Ptr JointDynamicsFromRR(const rr_sg::JointDynamicsPtr& joint_dynamics);

// Joint Limits

rr_sg::JointLimitsPtr JointLimitsToRR(const tesseract::scene_graph::JointLimits& joint_limits);

rr_sg::JointLimitsPtr JointLimitsToRR(const tesseract::scene_graph::JointLimits::ConstPtr joint_limits);

tesseract::scene_graph::JointLimits::Ptr JointLimitsFromRR(const rr_sg::JointLimitsPtr& joint_limits);

// Joint Safety

rr_sg::JointSafetyPtr JointSafetyToRR(const tesseract::scene_graph::JointSafety& joint_safety);

rr_sg::JointSafetyPtr JointSafetyToRR(const tesseract::scene_graph::JointSafety::ConstPtr joint_safety);

tesseract::scene_graph::JointSafety::Ptr JointSafetyFromRR(const rr_sg::JointSafetyPtr& joint_safety);

// JointCalibration

rr_sg::JointCalibrationPtr JointCalibrationToRR(const tesseract::scene_graph::JointCalibration& joint_calibration);

rr_sg::JointCalibrationPtr JointCalibrationToRR(
    const tesseract::scene_graph::JointCalibration::ConstPtr joint_calibration);

tesseract::scene_graph::JointCalibration::Ptr JointCalibrationFromRR(
    const rr_sg::JointCalibrationPtr& joint_calibration);

// JointMimic

rr_sg::JointMimicPtr JointMimicToRR(const tesseract::scene_graph::JointMimic& joint_mimic);

rr_sg::JointMimicPtr JointMimicToRR(const tesseract::scene_graph::JointMimic::ConstPtr joint_mimic);

tesseract::scene_graph::JointMimic::Ptr JointMimicFromRR(const rr_sg::JointMimicPtr& joint_mimic);

// Joint
rr_sg::JointPtr JointToRR(const tesseract::scene_graph::Joint& joint);

rr_sg::JointPtr JointToRR(const tesseract::scene_graph::Joint::ConstPtr joint);

tesseract::scene_graph::Joint::Ptr JointFromRR(const rr_sg::JointPtr& joint);

// Inertial using com.robotraconteur.geometry.SpatialInertia

rr_geom::SpatialInertia InertialToRR(const tesseract::scene_graph::Inertial& inertial);

rr_geom::SpatialInertia InertialToRR(const tesseract::scene_graph::Inertial::ConstPtr inertial);

tesseract::scene_graph::Inertial::Ptr InertialFromRR(const rr_geom::SpatialInertia& inertial);

// Link
rr_sg::LinkPtr LinkToRR(const tesseract::scene_graph::Link& link);

rr_sg::LinkPtr LinkToRR(const tesseract::scene_graph::Link::ConstPtr link);

tesseract::scene_graph::Link::Ptr LinkFromRR(const rr_sg::LinkPtr& link);

// SceneGraph
rr_sg::SceneGraphPtr SceneGraphToRR(const tesseract::scene_graph::SceneGraph& scene_graph);

rr_sg::SceneGraphPtr SceneGraphToRR(const tesseract::scene_graph::SceneGraph::ConstPtr scene_graph);

tesseract::scene_graph::SceneGraph::Ptr SceneGraphFromRR(const rr_sg::SceneGraphPtr& scene_graph);

// SceneState
rr_sg::SceneStatePtr SceneStateToRR(const tesseract::scene_graph::SceneState& scene_state);

} // namespace environment_conv
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_SCENE_GRAPH_CONV_H
