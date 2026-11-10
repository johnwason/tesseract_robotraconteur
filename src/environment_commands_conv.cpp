/**
 * @file environment_commands_conv.cpp
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
#include <tesseract_robotraconteur/conv/environment_commands_conv.h>

namespace RR = RobotRaconteur;
namespace RRC_Eigen = RobotRaconteur::Companion::Converters::Eigen;

using namespace tesseract_robotraconteur::conv;

#define RR_ENV_CMDS_PREFIX "experimental.tesseract_robotics.environment.commands"

namespace tesseract_robotraconteur
{
namespace environment_conv
{
// struct AddContactManagersPluginInfoCommand
//     # Placeholder only
//     field int32 command_id
// end

rr_env_cmd::AddContactManagersPluginInfoCommandPtr AddContactManagersPluginInfoCommandToRR(
    const tesseract::environment::AddContactManagersPluginInfoCommand::ConstPtr& cmd)
{
    rr_env_cmd::AddContactManagersPluginInfoCommandPtr rr_cmd(new rr_env_cmd::AddContactManagersPluginInfoCommand());
    return rr_cmd;
}

tesseract::environment::AddContactManagersPluginInfoCommand::Ptr AddContactManagersPluginInfoCommandFromRR(
    const rr_env_cmd::AddContactManagersPluginInfoCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::AddContactManagersPluginInfoCommand>();
    return ret;
}

// struct AddKinematicsInformationCommand
//     # TODO
//     field int32 noop
// end

rr_env_cmd::AddKinematicsInformationCommandPtr AddKinematicsInformationCommandToRR(
    const tesseract::environment::AddKinematicsInformationCommand::ConstPtr& cmd)
{
    rr_env_cmd::AddKinematicsInformationCommandPtr rr_cmd(new rr_env_cmd::AddKinematicsInformationCommand());
    return rr_cmd;
}

tesseract::environment::AddKinematicsInformationCommand::Ptr AddKinematicsInformationCommandFromRR(
    const rr_env_cmd::AddKinematicsInformationCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::AddKinematicsInformationCommand>();
    return ret;
}

// struct AddLinkCommand
//     field Link link
//     field Joint joint
//     field bool replace_allowed
// end

rr_env_cmd::AddLinkCommandPtr AddLinkCommandToRR(const tesseract::environment::AddLinkCommand::ConstPtr& cmd)
{
    rr_env_cmd::AddLinkCommandPtr rr_cmd(new rr_env_cmd::AddLinkCommand());
    rr_cmd->link = LinkToRR(cmd->getLink());
    rr_cmd->joint = JointToRR(cmd->getJoint());
    rr_cmd->replace_allowed = cmd->replaceAllowed();
    return rr_cmd;
}

tesseract::environment::AddLinkCommand::Ptr AddLinkCommandFromRR(const rr_env_cmd::AddLinkCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto link = LinkFromRR(cmd->link);
    auto joint = JointFromRR(cmd->joint);
    bool replace_allowed = cmd->replace_allowed.value != 0;
    return std::make_shared<tesseract::environment::AddLinkCommand>(*link, *joint, replace_allowed);
}

// struct AddSceneGraphCommand
//     field Link{list} links
//     field Joint{list} joints
//     field Joint joint
//     field string prefix
// end

rr_env_cmd::AddSceneGraphCommandPtr AddSceneGraphCommandToRR(
    const tesseract::environment::AddSceneGraphCommand::ConstPtr& cmd)
{
    rr_env_cmd::AddSceneGraphCommandPtr rr_cmd(new rr_env_cmd::AddSceneGraphCommand());
    rr_cmd->scene_graph = SceneGraphToRR(cmd->getSceneGraph());
    if (rr_cmd->joint)
    {
        rr_cmd->joint = JointToRR(cmd->getJoint());
    }
    rr_cmd->prefix = cmd->getPrefix();
    return rr_cmd;
}

tesseract::environment::AddSceneGraphCommand::Ptr AddSceneGraphCommandFromRR(
    const rr_env_cmd::AddSceneGraphCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    std::vector<tesseract::scene_graph::Link::Ptr> links;
    RR_NULL_CHECK(cmd->scene_graph);
    tesseract::scene_graph::SceneGraph::Ptr graph = SceneGraphFromRR(cmd->scene_graph);

    tesseract::scene_graph::Joint::Ptr joint;
    if (cmd->joint)
    {
        joint = JointFromRR(cmd->joint);
    }
    auto ret = std::make_shared<tesseract::environment::AddSceneGraphCommand>(*graph, *joint, cmd->prefix);
    return ret;
}

// struct AddTrajectoryLinkCommand
//     # TODO
//     field int32 noop
// end

rr_env_cmd::AddTrajectoryLinkCommandPtr AddTrajectoryLinkCommandToRR(
    const tesseract::environment::AddTrajectoryLinkCommand::ConstPtr& cmd)
{
    rr_env_cmd::AddTrajectoryLinkCommandPtr rr_cmd(new rr_env_cmd::AddTrajectoryLinkCommand());
    return rr_cmd;
}

tesseract::environment::AddTrajectoryLinkCommand::Ptr AddTrajectoryLinkCommandFromRR(
    const rr_env_cmd::AddTrajectoryLinkCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::AddTrajectoryLinkCommand>();
    return ret;
}

// struct ChangeCollisionMarginsCommand
//     # TODO
//     field int32 noop
// end

rr_env_cmd::ChangeCollisionMarginsCommandPtr ChangeCollisionMarginsCommandToRR(
    const tesseract::environment::ChangeCollisionMarginsCommand::ConstPtr& cmd)
{
    rr_env_cmd::ChangeCollisionMarginsCommandPtr rr_cmd(new rr_env_cmd::ChangeCollisionMarginsCommand());
    return rr_cmd;
}

tesseract::environment::ChangeCollisionMarginsCommand::Ptr ChangeCollisionMarginsCommandFromRR(
    const rr_env_cmd::ChangeCollisionMarginsCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::ChangeCollisionMarginsCommand>();
    return ret;
}

// struct ChangeJointAccelerationLimitsCommand
//     field double{string} limits
// end

rr_env_cmd::ChangeJointAccelerationLimitsCommandPtr ChangeJointAccelerationLimitsCommandToRR(
    const tesseract::environment::ChangeJointAccelerationLimitsCommand::ConstPtr& cmd)
{
    rr_env_cmd::ChangeJointAccelerationLimitsCommandPtr rr_cmd(new rr_env_cmd::ChangeJointAccelerationLimitsCommand());
    rr_cmd->limits = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double>>();
    for (const auto& limit : cmd->getLimits())
    {
        rr_cmd->limits->insert(std::make_pair(limit.first, RR::ScalarToRRArray<double>(limit.second)));
    }
    return rr_cmd;
}

tesseract::environment::ChangeJointAccelerationLimitsCommand::Ptr ChangeJointAccelerationLimitsCommandFromRR(
    const rr_env_cmd::ChangeJointAccelerationLimitsCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    std::unordered_map<std::string, double> limits;
    RR_NULL_CHECK(cmd->limits);
    for (const auto& limit : *cmd->limits)
    {
        limits.insert(std::make_pair(limit.first, RR::RRArrayToScalar<double>(limit.second)));
    }
    auto ret = std::make_shared<tesseract::environment::ChangeJointAccelerationLimitsCommand>(limits);
    return ret;
}

// struct ChangeJointOriginCommand
//     field string joint_name
//     field Pose origin
// end

rr_env_cmd::ChangeJointOriginCommandPtr ChangeJointOriginCommandToRR(
    const tesseract::environment::ChangeJointOriginCommand::ConstPtr& cmd)
{
    rr_env_cmd::ChangeJointOriginCommandPtr rr_cmd(new rr_env_cmd::ChangeJointOriginCommand());
    rr_cmd->joint_name = cmd->getJointName();
    rr_cmd->origin = RRC_Eigen::ToPose(cmd->getOrigin());
    return rr_cmd;
}

tesseract::environment::ChangeJointOriginCommand::Ptr ChangeJointOriginCommandFromRR(
    const rr_env_cmd::ChangeJointOriginCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::ChangeJointOriginCommand>(cmd->joint_name,
                                                                                  RRC_Eigen::ToIsometry(cmd->origin));
    return ret;
}

// struct ChangeJointPositionLimitsCommand
//     field double[2]{string} limits
// end

rr_env_cmd::ChangeJointPositionLimitsCommandPtr ChangeJointPositionLimitsCommandToRR(
    const tesseract::environment::ChangeJointPositionLimitsCommand::ConstPtr& cmd)
{
    rr_env_cmd::ChangeJointPositionLimitsCommandPtr rr_cmd(new rr_env_cmd::ChangeJointPositionLimitsCommand());
    rr_cmd->limits = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double>>();
    for (const auto& limit : cmd->getLimits())
    {
        auto val = RR::AllocateRRArray<double>(2);
        val->at(0) = limit.second.first;
        val->at(1) = limit.second.second;
        rr_cmd->limits->insert(std::make_pair(limit.first, val));
    }
    return rr_cmd;
}

tesseract::environment::ChangeJointPositionLimitsCommand::Ptr ChangeJointPositionLimitsCommandFromRR(
    const rr_env_cmd::ChangeJointPositionLimitsCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    std::unordered_map<std::string, std::pair<double, double>> limits;
    RR_NULL_CHECK(cmd->limits);
    for (const auto& limit : *cmd->limits)
    {
        limits.insert(std::make_pair(limit.first, std::make_pair(limit.second->at(0), limit.second->at(1))));
    }
    auto ret = std::make_shared<tesseract::environment::ChangeJointPositionLimitsCommand>(limits);
    return ret;
}

// struct ChangeJointVelocityLimitsCommand
//     field double{string} limits
// end

rr_env_cmd::ChangeJointVelocityLimitsCommandPtr ChangeJointVelocityLimitsCommandToRR(
    const tesseract::environment::ChangeJointVelocityLimitsCommand::ConstPtr& cmd)
{
    rr_env_cmd::ChangeJointVelocityLimitsCommandPtr rr_cmd(new rr_env_cmd::ChangeJointVelocityLimitsCommand());
    rr_cmd->limits = RR::AllocateEmptyRRMap<std::string, RR::RRArray<double>>();
    for (const auto& limit : cmd->getLimits())
    {
        rr_cmd->limits->insert(std::make_pair(limit.first, RR::ScalarToRRArray<double>(limit.second)));
    }
    return rr_cmd;
}

tesseract::environment::ChangeJointVelocityLimitsCommand::Ptr ChangeJointVelocityLimitsCommandFromRR(
    const rr_env_cmd::ChangeJointVelocityLimitsCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    std::unordered_map<std::string, double> limits;
    RR_NULL_CHECK(cmd->limits);
    for (const auto& limit : *cmd->limits)
    {
        limits.insert(std::make_pair(limit.first, RR::RRArrayToScalar<double>(limit.second)));
    }
    auto ret = std::make_shared<tesseract::environment::ChangeJointVelocityLimitsCommand>(limits);
    return ret;
}

// struct ChangeLinkCollisionEnabledCommand
//     field string link_name
//     field bool enabled
// end

rr_env_cmd::ChangeLinkCollisionEnabledCommandPtr ChangeLinkCollisionEnabledCommandToRR(
    const tesseract::environment::ChangeLinkCollisionEnabledCommand::ConstPtr& cmd)
{
    rr_env_cmd::ChangeLinkCollisionEnabledCommandPtr rr_cmd(new rr_env_cmd::ChangeLinkCollisionEnabledCommand());
    rr_cmd->link_name = cmd->getLinkName();
    rr_cmd->enabled = cmd->getEnabled();
    return rr_cmd;
}

tesseract::environment::ChangeLinkCollisionEnabledCommand::Ptr ChangeLinkCollisionEnabledCommandFromRR(
    const rr_env_cmd::ChangeLinkCollisionEnabledCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::ChangeLinkCollisionEnabledCommand>(cmd->link_name,
                                                                                           cmd->enabled.value != 0);
    return ret;
}

// struct ChangeLinkOriginCommand
//     field string link_name
//     field Pose origin
// end

rr_env_cmd::ChangeLinkOriginCommandPtr ChangeLinkOriginCommandToRR(
    const tesseract::environment::ChangeLinkOriginCommand::ConstPtr& cmd)
{
    rr_env_cmd::ChangeLinkOriginCommandPtr rr_cmd(new rr_env_cmd::ChangeLinkOriginCommand());
    rr_cmd->link_name = cmd->getLinkName();
    rr_cmd->origin = RRC_Eigen::ToPose(cmd->getOrigin());
    return rr_cmd;
}

tesseract::environment::ChangeLinkOriginCommand::Ptr ChangeLinkOriginCommandFromRR(
    const rr_env_cmd::ChangeLinkOriginCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::ChangeLinkOriginCommand>(cmd->link_name,
                                                                                 RRC_Eigen::ToIsometry(cmd->origin));
    return ret;
}

// struct ChangeLinkVisibilityCommand
//     field string link_name
//     field bool visible
// end

rr_env_cmd::ChangeLinkVisibilityCommandPtr ChangeLinkVisibilityCommandToRR(
    const tesseract::environment::ChangeLinkVisibilityCommand::ConstPtr& cmd)
{
    rr_env_cmd::ChangeLinkVisibilityCommandPtr rr_cmd(new rr_env_cmd::ChangeLinkVisibilityCommand());
    rr_cmd->link_name = cmd->getLinkName();
    rr_cmd->visible = cmd->getEnabled();
    return rr_cmd;
}

tesseract::environment::ChangeLinkVisibilityCommand::Ptr ChangeLinkVisibilityCommandFromRR(
    const rr_env_cmd::ChangeLinkVisibilityCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret =
        std::make_shared<tesseract::environment::ChangeLinkVisibilityCommand>(cmd->link_name, cmd->visible.value != 0);
    return ret;
}

// struct ModifyAllowedCollisionsCommand
//     # TODO
//     field int32 noop
// end

rr_env_cmd::ModifyAllowedCollisionsCommandPtr ModifyAllowedCollisionsCommandToRR(
    const tesseract::environment::ModifyAllowedCollisionsCommand::ConstPtr& cmd)
{
    rr_env_cmd::ModifyAllowedCollisionsCommandPtr rr_cmd(new rr_env_cmd::ModifyAllowedCollisionsCommand());
    return rr_cmd;
}

tesseract::environment::ModifyAllowedCollisionsCommand::Ptr ModifyAllowedCollisionsCommandFromRR(
    const rr_env_cmd::ModifyAllowedCollisionsCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::ModifyAllowedCollisionsCommand>();
    return ret;
}

// struct MoveJointCommand
//     field string joint_name
//     field string parent_link
// end

rr_env_cmd::MoveJointCommandPtr MoveJointCommandToRR(const tesseract::environment::MoveJointCommand::ConstPtr& cmd)
{
    rr_env_cmd::MoveJointCommandPtr rr_cmd(new rr_env_cmd::MoveJointCommand());
    rr_cmd->joint_name = cmd->getJointName();
    rr_cmd->parent_link = cmd->getParentLink();
    return rr_cmd;
}

tesseract::environment::MoveJointCommand::Ptr MoveJointCommandFromRR(const rr_env_cmd::MoveJointCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::MoveJointCommand>(cmd->joint_name, cmd->parent_link);
    return ret;
}

// struct MoveLinkCommand
//     field Joint joint
// end

rr_env_cmd::MoveLinkCommandPtr MoveLinkCommandToRR(const tesseract::environment::MoveLinkCommand::ConstPtr& cmd)
{
    rr_env_cmd::MoveLinkCommandPtr rr_cmd(new rr_env_cmd::MoveLinkCommand());
    rr_cmd->joint = JointToRR(cmd->getJoint());
    return rr_cmd;
}

tesseract::environment::MoveLinkCommand::Ptr MoveLinkCommandFromRR(const rr_env_cmd::MoveLinkCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto joint_ptr = JointFromRR(cmd->joint);
    auto ret = std::make_shared<tesseract::environment::MoveLinkCommand>(*joint_ptr);
    return ret;
}

// struct RemoveAllowedCollisionLinkCommand
//     # TODO
//     field int32 noop
// end

rr_env_cmd::RemoveAllowedCollisionLinkCommandPtr RemoveAllowedCollisionLinkCommandToRR(
    const tesseract::environment::RemoveAllowedCollisionLinkCommand::ConstPtr& cmd)
{
    rr_env_cmd::RemoveAllowedCollisionLinkCommandPtr rr_cmd(new rr_env_cmd::RemoveAllowedCollisionLinkCommand());
    return rr_cmd;
}

tesseract::environment::RemoveAllowedCollisionLinkCommand::Ptr RemoveAllowedCollisionLinkCommandFromRR(
    const rr_env_cmd::RemoveAllowedCollisionLinkCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::RemoveAllowedCollisionLinkCommand>();
    return ret;
}

// struct RemoveJointCommand
//     field string joint_name
// end

rr_env_cmd::RemoveJointCommandPtr RemoveJointCommandToRR(
    const tesseract::environment::RemoveJointCommand::ConstPtr& cmd)
{
    rr_env_cmd::RemoveJointCommandPtr rr_cmd(new rr_env_cmd::RemoveJointCommand());
    rr_cmd->joint_name = cmd->getJointName();
    return rr_cmd;
}

tesseract::environment::RemoveJointCommand::Ptr RemoveJointCommandFromRR(const rr_env_cmd::RemoveJointCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::RemoveJointCommand>(cmd->joint_name);
    return ret;
}

// struct RemoveLinkCommand
//     field string link_name
// end

rr_env_cmd::RemoveLinkCommandPtr RemoveLinkCommandToRR(const tesseract::environment::RemoveLinkCommand::ConstPtr& cmd)
{
    rr_env_cmd::RemoveLinkCommandPtr rr_cmd(new rr_env_cmd::RemoveLinkCommand());
    rr_cmd->link_name = cmd->getLinkName();
    return rr_cmd;
}

tesseract::environment::RemoveLinkCommand::Ptr RemoveLinkCommandFromRR(const rr_env_cmd::RemoveLinkCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::RemoveLinkCommand>(cmd->link_name);
    return ret;
}

// struct ReplaceJointCommand
//     field Joint joint
// end

rr_env_cmd::ReplaceJointCommandPtr ReplaceJointCommandToRR(
    const tesseract::environment::ReplaceJointCommand::ConstPtr& cmd)
{
    rr_env_cmd::ReplaceJointCommandPtr rr_cmd(new rr_env_cmd::ReplaceJointCommand());
    rr_cmd->joint = JointToRR(cmd->getJoint());
    return rr_cmd;
}

tesseract::environment::ReplaceJointCommand::Ptr ReplaceJointCommandFromRR(
    const rr_env_cmd::ReplaceJointCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto joint_ptr = JointFromRR(cmd->joint);
    auto ret = std::make_shared<tesseract::environment::ReplaceJointCommand>(*joint_ptr);
    return ret;
}

// struct SetActiveContinuousContactManagerCommand
//     field string active_contact_manager_name
// end

rr_env_cmd::SetActiveContinuousContactManagerCommandPtr SetActiveContinuousContactManagerCommandToRR(
    const tesseract::environment::SetActiveContinuousContactManagerCommand::ConstPtr& cmd)
{
    rr_env_cmd::SetActiveContinuousContactManagerCommandPtr rr_cmd(
        new rr_env_cmd::SetActiveContinuousContactManagerCommand());
    rr_cmd->active_contact_manager_name = cmd->getName();
    return rr_cmd;
}

tesseract::environment::SetActiveContinuousContactManagerCommand::Ptr SetActiveContinuousContactManagerCommandFromRR(
    const rr_env_cmd::SetActiveContinuousContactManagerCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::SetActiveContinuousContactManagerCommand>(
        cmd->active_contact_manager_name);
    return ret;
}

// struct SetActiveDiscreteContactManagerCommand
//     field string active_contact_manager_name
// end

rr_env_cmd::SetActiveDiscreteContactManagerCommandPtr SetActiveDiscreteContactManagerCommandToRR(
    const tesseract::environment::SetActiveDiscreteContactManagerCommand::ConstPtr& cmd)
{
    rr_env_cmd::SetActiveDiscreteContactManagerCommandPtr rr_cmd(
        new rr_env_cmd::SetActiveDiscreteContactManagerCommand());
    rr_cmd->active_contact_manager_name = cmd->getName();
    return rr_cmd;
}

tesseract::environment::SetActiveDiscreteContactManagerCommand::Ptr SetActiveDiscreteContactManagerCommandFromRR(
    const rr_env_cmd::SetActiveDiscreteContactManagerCommandPtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto ret = std::make_shared<tesseract::environment::SetActiveDiscreteContactManagerCommand>(
        cmd->active_contact_manager_name);
    return ret;
}

// Command
RobotRaconteur::RRValuePtr CommandToRR(const tesseract::environment::Command::ConstPtr& cmd)
{
    switch (cmd->getType())
    {
    case tesseract::environment::CommandType::ADD_LINK:
        return AddLinkCommandToRR(std::static_pointer_cast<const tesseract::environment::AddLinkCommand>(cmd));
    case tesseract::environment::CommandType::MOVE_LINK:
        return MoveLinkCommandToRR(std::static_pointer_cast<const tesseract::environment::MoveLinkCommand>(cmd));
    case tesseract::environment::CommandType::MOVE_JOINT:
        return MoveJointCommandToRR(std::static_pointer_cast<const tesseract::environment::MoveJointCommand>(cmd));
    case tesseract::environment::CommandType::REMOVE_LINK:
        return RemoveLinkCommandToRR(std::static_pointer_cast<const tesseract::environment::RemoveLinkCommand>(cmd));
    case tesseract::environment::CommandType::REMOVE_JOINT:
        return RemoveJointCommandToRR(std::static_pointer_cast<const tesseract::environment::RemoveJointCommand>(cmd));
    case tesseract::environment::CommandType::CHANGE_LINK_ORIGIN:
        return ChangeLinkOriginCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ChangeLinkOriginCommand>(cmd));
    case tesseract::environment::CommandType::CHANGE_JOINT_ORIGIN:
        return ChangeJointOriginCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ChangeJointOriginCommand>(cmd));
    case tesseract::environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
        return ChangeLinkCollisionEnabledCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ChangeLinkCollisionEnabledCommand>(cmd));
    case tesseract::environment::CommandType::CHANGE_LINK_VISIBILITY:
        return ChangeLinkVisibilityCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ChangeLinkVisibilityCommand>(cmd));
    case tesseract::environment::CommandType::MODIFY_ALLOWED_COLLISIONS:
        return ModifyAllowedCollisionsCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ModifyAllowedCollisionsCommand>(cmd));
    case tesseract::environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
        return RemoveAllowedCollisionLinkCommandToRR(
            std::static_pointer_cast<const tesseract::environment::RemoveAllowedCollisionLinkCommand>(cmd));
    case tesseract::environment::CommandType::ADD_SCENE_GRAPH:
        return AddSceneGraphCommandToRR(
            std::static_pointer_cast<const tesseract::environment::AddSceneGraphCommand>(cmd));
    case tesseract::environment::CommandType::CHANGE_JOINT_POSITION_LIMITS:
        return ChangeJointPositionLimitsCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ChangeJointPositionLimitsCommand>(cmd));
    case tesseract::environment::CommandType::CHANGE_JOINT_VELOCITY_LIMITS:
        return ChangeJointVelocityLimitsCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ChangeJointVelocityLimitsCommand>(cmd));
    case tesseract::environment::CommandType::CHANGE_JOINT_ACCELERATION_LIMITS:
        return ChangeJointAccelerationLimitsCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ChangeJointAccelerationLimitsCommand>(cmd));
    case tesseract::environment::CommandType::ADD_KINEMATICS_INFORMATION:
        return AddKinematicsInformationCommandToRR(
            std::static_pointer_cast<const tesseract::environment::AddKinematicsInformationCommand>(cmd));
    case tesseract::environment::CommandType::REPLACE_JOINT:
        return ReplaceJointCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ReplaceJointCommand>(cmd));
    case tesseract::environment::CommandType::CHANGE_COLLISION_MARGINS:
        return ChangeCollisionMarginsCommandToRR(
            std::static_pointer_cast<const tesseract::environment::ChangeCollisionMarginsCommand>(cmd));
    case tesseract::environment::CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO:
        return AddContactManagersPluginInfoCommandToRR(
            std::static_pointer_cast<const tesseract::environment::AddContactManagersPluginInfoCommand>(cmd));
    case tesseract::environment::CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER:
        return SetActiveDiscreteContactManagerCommandToRR(
            std::static_pointer_cast<const tesseract::environment::SetActiveDiscreteContactManagerCommand>(cmd));
    case tesseract::environment::CommandType::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER:
        return SetActiveContinuousContactManagerCommandToRR(
            std::static_pointer_cast<const tesseract::environment::SetActiveContinuousContactManagerCommand>(cmd));
    case tesseract::environment::CommandType::ADD_TRAJECTORY_LINK:
        return AddTrajectoryLinkCommandToRR(
            std::static_pointer_cast<const tesseract::environment::AddTrajectoryLinkCommand>(cmd));
    default:
        throw RR::InvalidArgumentException("Invalid command type");
    }
}

tesseract::environment::Command::Ptr CommandFromRR(const RobotRaconteur::RRValuePtr& cmd)
{
    RR_NULL_CHECK(cmd);
    auto cmd_type = cmd->RRType();

    if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                       "AddLinkCommand")
    {
        return AddLinkCommandFromRR(RR::rr_cast<rr_env_cmd::AddLinkCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "MoveLinkCommand")
    {
        return MoveLinkCommandFromRR(RR::rr_cast<rr_env_cmd::MoveLinkCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "MoveJointCommand")
    {
        return MoveJointCommandFromRR(RR::rr_cast<rr_env_cmd::MoveJointCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "RemoveLinkCommand")
    {
        return RemoveLinkCommandFromRR(RR::rr_cast<rr_env_cmd::RemoveLinkCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "RemoveJointCommand")
    {
        return RemoveJointCommandFromRR(RR::rr_cast<rr_env_cmd::RemoveJointCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ChangeLinkOriginCommand")
    {
        return ChangeLinkOriginCommandFromRR(RR::rr_cast<rr_env_cmd::ChangeLinkOriginCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ChangeJointOriginCommand")
    {
        return ChangeJointOriginCommandFromRR(RR::rr_cast<rr_env_cmd::ChangeJointOriginCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ChangeLinkCollisionEnabledCommand")
    {
        return ChangeLinkCollisionEnabledCommandFromRR(RR::rr_cast<rr_env_cmd::ChangeLinkCollisionEnabledCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ChangeLinkVisibilityCommand")
    {
        return ChangeLinkVisibilityCommandFromRR(RR::rr_cast<rr_env_cmd::ChangeLinkVisibilityCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ModifyAllowedCollisionsCommand")
    {
        return ModifyAllowedCollisionsCommandFromRR(RR::rr_cast<rr_env_cmd::ModifyAllowedCollisionsCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "RemoveAllowedCollisionLinkCommand")
    {
        return RemoveAllowedCollisionLinkCommandFromRR(RR::rr_cast<rr_env_cmd::RemoveAllowedCollisionLinkCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "AddSceneGraphCommand")
    {
        return AddSceneGraphCommandFromRR(RR::rr_cast<rr_env_cmd::AddSceneGraphCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ChangeJointPositionLimitsCommand")
    {
        return ChangeJointPositionLimitsCommandFromRR(RR::rr_cast<rr_env_cmd::ChangeJointPositionLimitsCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ChangeJointVelocityLimitsCommand")
    {
        return ChangeJointVelocityLimitsCommandFromRR(RR::rr_cast<rr_env_cmd::ChangeJointVelocityLimitsCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ChangeJointAccelerationLimitsCommand")
    {
        return ChangeJointAccelerationLimitsCommandFromRR(
            RR::rr_cast<rr_env_cmd::ChangeJointAccelerationLimitsCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "AddKinematicsInformationCommand")
    {
        return AddKinematicsInformationCommandFromRR(RR::rr_cast<rr_env_cmd::AddKinematicsInformationCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ReplaceJointCommand")
    {
        return ReplaceJointCommandFromRR(RR::rr_cast<rr_env_cmd::ReplaceJointCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "ChangeCollisionMarginsCommand")
    {
        return ChangeCollisionMarginsCommandFromRR(RR::rr_cast<rr_env_cmd::ChangeCollisionMarginsCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "AddContactManagersPluginInfoCommand")
    {
        return AddContactManagersPluginInfoCommandFromRR(
            RR::rr_cast<rr_env_cmd::AddContactManagersPluginInfoCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "SetActiveDiscreteContactManagerCommand")
    {
        return SetActiveDiscreteContactManagerCommandFromRR(
            RR::rr_cast<rr_env_cmd::SetActiveDiscreteContactManagerCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "SetActiveContinuousContactManagerCommand")
    {
        return SetActiveContinuousContactManagerCommandFromRR(
            RR::rr_cast<rr_env_cmd::SetActiveContinuousContactManagerCommand>(cmd));
    }
    else if (cmd_type == RR_ENV_CMDS_PREFIX "."
                                            "AddTrajectoryLinkCommand")
    {
        return AddTrajectoryLinkCommandFromRR(RR::rr_cast<rr_env_cmd::AddTrajectoryLinkCommand>(cmd));
    }
    else
    {
        throw RR::InvalidArgumentException("Invalid environment command type");
    }
}

// Commands
rr_env_cmd::CommandsPtr CommandsToRR(const tesseract::environment::Commands& commands)
{
    rr_env_cmd::CommandsPtr rr_commands(new rr_env_cmd::Commands());
    rr_commands->commands = RR::AllocateEmptyRRList<RR::RRValue>();
    for (const auto& cmd : commands)
    {
        rr_commands->commands->push_back(CommandToRR(cmd));
    }
    return rr_commands;
}

tesseract::environment::Commands CommandsFromRR(const rr_env_cmd::CommandsPtr& commands)
{
    RR_NULL_CHECK(commands);
    tesseract::environment::Commands ret;
    RR_NULL_CHECK(commands->commands);
    for (const auto& cmd : *commands->commands)
    {
        ret.push_back(CommandFromRR(cmd));
    }
    return ret;
}

} // namespace environment_conv
} // namespace tesseract_robotraconteur
