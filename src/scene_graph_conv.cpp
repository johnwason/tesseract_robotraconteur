/**
 * @file scene_graph_conv.cpp
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

#include "tesseract_robotraconteur/conv/environment_commands_conv.h"

#include <RobotRaconteurCompanion/Converters/EigenConverters.h>

#include <tesseract_robotraconteur/conv/common_conv.h>
#include <tesseract_robotraconteur/conv/geometry_conv.h>
#include <tesseract_robotraconteur/conv/scene_graph_conv.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>

#include <boost/range/algorithm.hpp>

namespace RR = RobotRaconteur;
namespace RRC_Eigen = RobotRaconteur::Companion::Converters::Eigen;

using namespace tesseract_robotraconteur::conv;

#define RR_ENV_PREFIX "experimental.tesseract_robotics.environment"

namespace tesseract_robotraconteur
{
namespace environment_conv
{

// Visual
rr_sg::VisualPtr VisualToRR(const tesseract::scene_graph::Visual& visual)
{
    rr_sg::VisualPtr ret(new rr_sg::Visual());
    ret->name = visual.name;
    ret->origin = RRC_Eigen::ToPose(visual.origin);
    ret->geometry = GeometryToRR(*visual.geometry);
    ret->material = LinkMaterialToRR(visual.material, visual.geometry);
    return ret;
}

rr_sg::VisualPtr VisualToRR(const tesseract::scene_graph::Visual::ConstPtr& visual) { return VisualToRR(*visual); }

tesseract::scene_graph::Visual::Ptr VisualFromRR(const rr_sg::VisualPtr& visual)
{
    RR_NULL_CHECK(visual);
    auto ret = std::make_shared<tesseract::scene_graph::Visual>();
    ret->name = visual->name;
    ret->origin = RRC_Eigen::ToIsometry(visual->origin);
    ret->geometry = GeometryFromRR(visual->geometry);
    ret->material = LinkMaterialFromRR(visual->material);
    return ret;
}

// LinkMaterial
rr_shapes::MaterialPtr LinkMaterialToRR(const tesseract::scene_graph::Material& link_material,
                                        const std::shared_ptr<const tesseract::geometry::Geometry>& geom)
{
    if (geom)
    {
        switch (geom->getType())
        {
        case tesseract::geometry::GeometryType::MESH:
        case tesseract::geometry::GeometryType::POLYGON_MESH:
        case tesseract::geometry::GeometryType::SDF_MESH:
        case tesseract::geometry::GeometryType::CONVEX_MESH: {
            auto mesh = static_cast<const tesseract::geometry::PolygonMesh&>(*geom);
            const auto mesh_material = mesh.getMaterial();
            if (mesh_material)
            {
                rr_shapes::MaterialPtr ret(new rr_shapes::Material());
                auto base_color = mesh_material->getBaseColorFactor();
                auto emissive = mesh_material->getEmissiveFactor();
                ret->base_color_factor.s.r = base_color(0);
                ret->base_color_factor.s.g = base_color(1);
                ret->base_color_factor.s.b = base_color(2);
                ret->base_color_factor.s.a = base_color(3);
                ret->emissive_factor.s.r = emissive(0);
                ret->emissive_factor.s.g = emissive(1);
                ret->emissive_factor.s.b = emissive(2);
                ret->emissive_factor.s.a = emissive(3);
                ret->metallic_factor = mesh_material->getMetallicFactor();
                ret->roughness_factor = mesh_material->getRoughnessFactor();
                return ret;
            }
        }
        default:
            break;
        }
    }
    rr_shapes::MaterialPtr ret(new rr_shapes::Material());
    ret->base_color_factor.s.r = link_material.color(0);
    ret->base_color_factor.s.g = link_material.color(1);
    ret->base_color_factor.s.b = link_material.color(2);
    ret->base_color_factor.s.a = link_material.color(3);
    return ret;
}

rr_shapes::MaterialPtr LinkMaterialToRR(const tesseract::scene_graph::Material::ConstPtr& link_material,
                                        const std::shared_ptr<const tesseract::geometry::Geometry>& geom)
{
    return LinkMaterialToRR(*link_material, geom);
}

tesseract::scene_graph::Material::Ptr LinkMaterialFromRR(const rr_shapes::MaterialPtr& link_material)
{
    RR_NULL_CHECK(link_material);
    auto ret = std::make_shared<tesseract::scene_graph::Material>();
    ret->color(0) = link_material->base_color_factor.s.r;
    ret->color(1) = link_material->base_color_factor.s.g;
    ret->color(2) = link_material->base_color_factor.s.b;
    ret->color(3) = link_material->base_color_factor.s.a;
    return ret;
}

// Collision

rr_sg::CollisionPtr CollisionToRR(const tesseract::scene_graph::Collision& collision)
{
    rr_sg::CollisionPtr ret(new rr_sg::Collision());
    ret->name = collision.name;
    ret->origin = RRC_Eigen::ToPose(collision.origin);
    ret->geometry = GeometryToRR(*collision.geometry);
    return ret;
}

rr_sg::CollisionPtr CollisionToRR(const tesseract::scene_graph::Collision::ConstPtr& collision)
{
    return CollisionToRR(*collision);
}

tesseract::scene_graph::Collision::Ptr CollisionFromRR(const rr_sg::CollisionPtr& collision)
{
    RR_NULL_CHECK(collision);
    auto ret = std::make_shared<tesseract::scene_graph::Collision>();
    ret->name = collision->name;
    ret->origin = RRC_Eigen::ToIsometry(collision->origin);
    ret->geometry = GeometryFromRR(collision->geometry);
    return ret;
}

// Joint Dynamics

rr_sg::JointDynamicsPtr JointDynamicsToRR(const tesseract::scene_graph::JointDynamics& joint_dynamics)
{
    rr_sg::JointDynamicsPtr ret(new rr_sg::JointDynamics());
    ret->damping = joint_dynamics.damping;
    ret->friction = joint_dynamics.friction;
    return ret;
}

rr_sg::JointDynamicsPtr JointDynamicsToRR(const tesseract::scene_graph::JointDynamics::ConstPtr joint_dynamics)
{
    return JointDynamicsToRR(*joint_dynamics);
}

tesseract::scene_graph::JointDynamics::Ptr JointDynamicsFromRR(const rr_sg::JointDynamicsPtr& joint_dynamics)
{
    RR_NULL_CHECK(joint_dynamics);
    auto ret = std::make_shared<tesseract::scene_graph::JointDynamics>();
    ret->damping = joint_dynamics->damping;
    ret->friction = joint_dynamics->friction;
    return ret;
}

// Joint Limits

rr_sg::JointLimitsPtr JointLimitsToRR(const tesseract::scene_graph::JointLimits& joint_limits)
{
    rr_sg::JointLimitsPtr ret(new rr_sg::JointLimits());
    ret->lower = joint_limits.lower;
    ret->upper = joint_limits.upper;
    ret->velocity = joint_limits.velocity;
    ret->acceleration = joint_limits.acceleration;
    ret->effort = joint_limits.effort;
    return ret;
}

rr_sg::JointLimitsPtr JointLimitsToRR(const tesseract::scene_graph::JointLimits::ConstPtr joint_limits)
{
    return JointLimitsToRR(*joint_limits);
}

tesseract::scene_graph::JointLimits::Ptr JointLimitsFromRR(const rr_sg::JointLimitsPtr& joint_limits)
{
    RR_NULL_CHECK(joint_limits);
    auto ret = std::make_shared<tesseract::scene_graph::JointLimits>();
    ret->lower = joint_limits->lower;
    ret->upper = joint_limits->upper;
    ret->velocity = joint_limits->velocity;
    ret->acceleration = joint_limits->acceleration;
    ret->effort = joint_limits->effort;
    return ret;
}

// Joint Safety

rr_sg::JointSafetyPtr JointSafetyToRR(const tesseract::scene_graph::JointSafety& joint_safety)
{
    rr_sg::JointSafetyPtr ret(new rr_sg::JointSafety());
    ret->soft_lower_limit = joint_safety.soft_lower_limit;
    ret->soft_upper_limit = joint_safety.soft_upper_limit;
    ret->k_position = joint_safety.k_position;
    ret->k_velocity = joint_safety.k_velocity;
    return ret;
}

rr_sg::JointSafetyPtr JointSafetyToRR(const tesseract::scene_graph::JointSafety::ConstPtr joint_safety)
{
    return JointSafetyToRR(*joint_safety);
}

tesseract::scene_graph::JointSafety::Ptr JointSafetyFromRR(const rr_sg::JointSafetyPtr& joint_safety)
{
    RR_NULL_CHECK(joint_safety);
    auto ret = std::make_shared<tesseract::scene_graph::JointSafety>();
    ret->soft_lower_limit = joint_safety->soft_lower_limit;
    ret->soft_upper_limit = joint_safety->soft_upper_limit;
    ret->k_position = joint_safety->k_position;
    ret->k_velocity = joint_safety->k_velocity;
    return ret;
}

// JointCalibration

rr_sg::JointCalibrationPtr JointCalibrationToRR(const tesseract::scene_graph::JointCalibration& joint_calibration)
{
    rr_sg::JointCalibrationPtr ret(new rr_sg::JointCalibration());
    ret->reference_position = joint_calibration.reference_position;
    ret->rising = joint_calibration.rising;
    ret->falling = joint_calibration.falling;
    return ret;
}

rr_sg::JointCalibrationPtr JointCalibrationToRR(
    const tesseract::scene_graph::JointCalibration::ConstPtr joint_calibration)
{
    return JointCalibrationToRR(*joint_calibration);
}

tesseract::scene_graph::JointCalibration::Ptr JointCalibrationFromRR(
    const rr_sg::JointCalibrationPtr& joint_calibration)
{
    RR_NULL_CHECK(joint_calibration);
    auto ret = std::make_shared<tesseract::scene_graph::JointCalibration>();
    ret->reference_position = joint_calibration->reference_position;
    ret->rising = joint_calibration->rising;
    ret->falling = joint_calibration->falling;
    return ret;
}

// JointMimic

rr_sg::JointMimicPtr JointMimicToRR(const tesseract::scene_graph::JointMimic& joint_mimic)
{
    rr_sg::JointMimicPtr ret(new rr_sg::JointMimic());
    ret->joint_name = joint_mimic.joint_name;
    ret->multiplier = joint_mimic.multiplier;
    ret->offset = joint_mimic.offset;
    return ret;
}

rr_sg::JointMimicPtr JointMimicToRR(const tesseract::scene_graph::JointMimic::ConstPtr joint_mimic)
{
    return JointMimicToRR(*joint_mimic);
}

tesseract::scene_graph::JointMimic::Ptr JointMimicFromRR(const rr_sg::JointMimicPtr& joint_mimic)
{
    RR_NULL_CHECK(joint_mimic);
    auto ret = std::make_shared<tesseract::scene_graph::JointMimic>();
    ret->joint_name = joint_mimic->joint_name;
    ret->multiplier = joint_mimic->multiplier;
    ret->offset = joint_mimic->offset;
    return ret;
}

// Joint
rr_sg::JointPtr JointToRR(const tesseract::scene_graph::Joint& joint)
{
    rr_sg::JointPtr ret(new rr_sg::Joint());
    ret->name = joint.getName();
    ret->joint_type = (rr_sg::JointType::JointType)joint.type;
    ret->parent_link_name = joint.parent_link_name;
    ret->child_link_name = joint.child_link_name;
    ret->parent_to_joint_origin_transform = RRC_Eigen::ToPose(joint.parent_to_joint_origin_transform);
    ret->axis = RRC_Eigen::ToVector3(joint.axis);
    if (joint.limits)
        ret->limits = JointLimitsToRR(*joint.limits);
    if (joint.dynamics)
        ret->dynamics = JointDynamicsToRR(*joint.dynamics);
    if (joint.safety)
        ret->safety = JointSafetyToRR(*joint.safety);
    if (joint.calibration)
        ret->calibration = JointCalibrationToRR(*joint.calibration);
    if (joint.mimic)
        ret->mimic = JointMimicToRR(*joint.mimic);
    return ret;
}

rr_sg::JointPtr JointToRR(const tesseract::scene_graph::Joint::ConstPtr joint) { return JointToRR(*joint); }

tesseract::scene_graph::Joint::Ptr JointFromRR(const rr_sg::JointPtr& joint)
{
    RR_NULL_CHECK(joint);
    auto ret = std::make_shared<tesseract::scene_graph::Joint>(joint->name);
    ret->type = (tesseract::scene_graph::JointType)joint->joint_type;
    ret->parent_link_name = joint->parent_link_name;
    ret->child_link_name = joint->child_link_name;
    ret->parent_to_joint_origin_transform = RRC_Eigen::ToIsometry(joint->parent_to_joint_origin_transform);
    ret->axis = RRC_Eigen::ToVector(joint->axis);
    if (joint->limits)
        ret->limits = JointLimitsFromRR(joint->limits);
    if (joint->dynamics)
        ret->dynamics = JointDynamicsFromRR(joint->dynamics);
    if (joint->safety)
        ret->safety = JointSafetyFromRR(joint->safety);
    if (joint->calibration)
        ret->calibration = JointCalibrationFromRR(joint->calibration);
    if (joint->mimic)
        ret->mimic = JointMimicFromRR(joint->mimic);
    return ret;
}

// Inertial using com.robotraconteur.geometry.SpatialInertia

rr_geom::SpatialInertia InertialToRR(const tesseract::scene_graph::Inertial& inertial)
{
    rr_geom::SpatialInertia ret;
    ret.s.m = inertial.mass;
    // TODO: handle rotation of inertia
    ret.s.com = RRC_Eigen::ToVector3(inertial.origin.translation());
    ret.s.ixx = inertial.ixx;
    ret.s.ixy = inertial.ixy;
    ret.s.ixz = inertial.ixz;
    ret.s.iyy = inertial.iyy;
    ret.s.iyz = inertial.iyz;
    ret.s.izz = inertial.izz;
    return ret;
}

rr_geom::SpatialInertia InertialToRR(const tesseract::scene_graph::Inertial::ConstPtr inertial)
{
    return InertialToRR(*inertial);
}

tesseract::scene_graph::Inertial::Ptr InertialFromRR(const rr_geom::SpatialInertia& inertial)
{
    auto ret = std::make_shared<tesseract::scene_graph::Inertial>();
    ret->mass = inertial.s.m;
    ret->origin = Eigen::Isometry3d::Identity() * Eigen::Translation3d(RRC_Eigen::ToVector(inertial.s.com));
    ret->ixx = inertial.s.ixx;
    ret->ixy = inertial.s.ixy;
    ret->ixz = inertial.s.ixz;
    ret->iyy = inertial.s.iyy;
    ret->iyz = inertial.s.iyz;
    ret->izz = inertial.s.izz;
    return ret;
}

// Link
rr_sg::LinkPtr LinkToRR(const tesseract::scene_graph::Link& link)
{
    rr_sg::LinkPtr ret(new rr_sg::Link());
    ret->name = link.getName();
    if (link.inertial)
    {
        ret->inertial = InertialToRR(*link.inertial);
    }
    ret->visual = RR::AllocateEmptyRRList<rr_sg::Visual>();
    for (auto& visual : link.visual)
    {
        if (visual->geometry)
        {
            if (visual->geometry->getType() == tesseract::geometry::GeometryType::COMPOUND_MESH)
            {
                auto& meshes = static_cast<const tesseract::geometry::CompoundMesh&>(*visual->geometry).getMeshes();
                size_t submesh_count = 0;
                for (const auto& m : meshes)
                {
                    if (!m)
                    {
                        continue;
                    }
                    auto rr_m = std::get<0>(MeshToRR(*m));
                    rr_sg::VisualPtr visual_ptr1(new rr_sg::Visual());
                    visual_ptr1->name = visual->name + "_submesh" + boost::lexical_cast<std::string>(submesh_count);
                    visual_ptr1->origin = RRC_Eigen::ToPose(visual->origin);
                    visual_ptr1->geometry = rr_m;
                    visual_ptr1->material = LinkMaterialToRR(visual->material, m);
                    ret->visual->push_back(visual_ptr1);
                }
                continue;
            }
        }
        auto visual_ptr = VisualToRR(visual);
        ret->visual->push_back(visual_ptr);
    }
    ret->collision = RR::AllocateEmptyRRList<rr_sg::Collision>();
    for (auto& collision : link.collision)
    {
        auto collision_ptr = CollisionToRR(collision);
        ret->collision->push_back(collision_ptr);
    }
    return ret;
}

rr_sg::LinkPtr LinkToRR(const tesseract::scene_graph::Link::ConstPtr link) { return LinkToRR(*link); }

tesseract::scene_graph::Link::Ptr LinkFromRR(const rr_sg::LinkPtr& link)
{
    RR_NULL_CHECK(link);
    auto ret = std::make_shared<tesseract::scene_graph::Link>(link->name);
    ret->inertial = InertialFromRR(link->inertial);
    RR_NULL_CHECK(link->visual);
    for (auto& visual : *link->visual)
    {
        auto visual_ptr = VisualFromRR(visual);
        ret->visual.push_back(visual_ptr);
    }
    RR_NULL_CHECK(link->collision);
    for (auto& collision : *link->collision)
    {
        auto collision_ptr = CollisionFromRR(collision);
        ret->collision.push_back(collision_ptr);
    }
    return ret;
}

// SceneGraph
rr_sg::SceneGraphPtr SceneGraphToRR(const tesseract::scene_graph::SceneGraph& scene_graph)
{
    rr_sg::SceneGraphPtr ret(new rr_sg::SceneGraph());
    ret->name = scene_graph.getName();
    ret->links = RR::AllocateEmptyRRList<rr_sg::Link>();
    for (auto& link : scene_graph.getLinks())
    {
        auto link_ptr = LinkToRR(link);
        ret->links->push_back(link_ptr);
    }
    ret->joints = RR::AllocateEmptyRRList<rr_sg::Joint>();
    for (auto& joint : scene_graph.getJoints())
    {
        auto joint_ptr = JointToRR(joint);
        ret->joints->push_back(joint_ptr);
    }
    return ret;
}

rr_sg::SceneGraphPtr SceneGraphToRR(const tesseract::scene_graph::SceneGraph::ConstPtr scene_graph)
{
    return SceneGraphToRR(*scene_graph);
}

tesseract::scene_graph::SceneGraph::Ptr SceneGraphFromRR(const rr_sg::SceneGraphPtr& scene_graph)
{
    RR_NULL_CHECK(scene_graph);
    auto ret = std::make_shared<tesseract::scene_graph::SceneGraph>(scene_graph->name);
    RR_NULL_CHECK(scene_graph->links);
    RR_NULL_CHECK(scene_graph->joints);
    for (auto& link : *scene_graph->links)
    {
        auto link_ptr = LinkFromRR(link);
        ret->addLink(*link_ptr);
    }

    for (auto& joint : *scene_graph->joints)
    {
        auto joint_ptr = JointFromRR(joint);
        ret->addJoint(*joint_ptr);
    }

    return ret;
}

static void _fill_map(const std::unordered_map<std::string, double>& dbl_map,
                      RR::RRMapPtr<std::string, RR::RRArray<double>>& rr_map)
{
    for (const auto& e : dbl_map)
    {
        rr_map->insert(std::make_pair(e.first, RR::ScalarToRRArray(e.second)));
    }
}

static void _fill_map(const tesseract::common::TransformMap& ft_map,
                      RR::RRMapPtr<std::string, RR::RRNamedArray<rr_geom::Transform>>& rr_map)
{
    for (const auto& e : ft_map)
    {
        auto rr_t = RR::AllocateEmptyRRNamedArray<rr_geom::Transform>(1);
        rr_t->at(0) = RobotRaconteur::Companion::Converters::Eigen::ToTransform(e.second);
        rr_map->insert(std::make_pair(e.first, rr_t));
    }
}

// SceneState
rr_sg::SceneStatePtr SceneStateToRR(const tesseract::scene_graph::SceneState& scene_state)
{
    rr_sg::SceneStatePtr ret(new rr_sg::SceneState());
    ret->joints = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double>>();
    ret->floating_joints = RR::AllocateEmptyRRMap<std::string, RR::RRNamedArray<rr_geom::Transform>>();
    ret->link_transforms = RR::AllocateEmptyRRMap<std::string, RR::RRNamedArray<rr_geom::Transform>>();
    ret->joint_transforms = RR::AllocateEmptyRRMap<std::string, RR::RRNamedArray<rr_geom::Transform>>();

    _fill_map(scene_state.joints, ret->joints);
    _fill_map(scene_state.floating_joints, ret->floating_joints);
    _fill_map(scene_state.joint_transforms, ret->joint_transforms);
    _fill_map(scene_state.link_transforms, ret->link_transforms);

    return ret;
}

} // namespace environment_conv
} // namespace tesseract_robotraconteur
