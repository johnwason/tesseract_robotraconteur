/**
 * @file environment_commands_conv.h
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

#ifndef TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_COMMANDS_CONV_H
#define TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_COMMANDS_CONV_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "robotraconteur_generated.h"

#include <tesseract/environment/commands.h>
#include "scene_graph_conv.h"

namespace rr_env_cmd = experimental::tesseract_robotics::environment::commands;
namespace rr_sg = experimental::tesseract_robotics::scene_graph;

namespace tesseract_robotraconteur
{
namespace environment_conv
{
// AddContactManagersPluginInfoCommand
rr_env_cmd::AddContactManagersPluginInfoCommandPtr AddContactManagersPluginInfoCommandToRR(
    const tesseract::environment::AddContactManagersPluginInfoCommand::ConstPtr& cmd);

tesseract::environment::AddContactManagersPluginInfoCommand::Ptr AddContactManagersPluginInfoCommandFromRR(
    const rr_env_cmd::AddContactManagersPluginInfoCommandPtr& cmd);

// AddKinematicsInformationCommand
rr_env_cmd::AddKinematicsInformationCommandPtr AddKinematicsInformationCommandToRR(
    const tesseract::environment::AddKinematicsInformationCommand::ConstPtr& cmd);

tesseract::environment::AddKinematicsInformationCommand::Ptr AddKinematicsInformationCommandFromRR(
    const rr_env_cmd::AddKinematicsInformationCommandPtr& cmd);

// AddLinkCommand
rr_env_cmd::AddLinkCommandPtr AddLinkCommandToRR(const tesseract::environment::AddLinkCommand::ConstPtr& cmd);

tesseract::environment::AddLinkCommand::Ptr AddLinkCommandFromRR(const rr_env_cmd::AddLinkCommandPtr& cmd);

// AddSceneGraphCommand
rr_env_cmd::AddSceneGraphCommandPtr AddSceneGraphCommandToRR(
    const tesseract::environment::AddSceneGraphCommand::ConstPtr& cmd);

tesseract::environment::AddSceneGraphCommand::Ptr AddSceneGraphCommandFromRR(
    const rr_env_cmd::AddSceneGraphCommandPtr& cmd);

// AddTrajectoryLinkCommand
rr_env_cmd::AddTrajectoryLinkCommandPtr AddTrajectoryLinkCommandToRR(
    const tesseract::environment::AddTrajectoryLinkCommand::ConstPtr& cmd);

tesseract::environment::AddTrajectoryLinkCommand::Ptr AddTrajectoryLinkCommandFromRR(
    const rr_env_cmd::AddTrajectoryLinkCommandPtr& cmd);

// ChangeCollisionMarginsCommand
rr_env_cmd::ChangeCollisionMarginsCommandPtr ChangeCollisionMarginsCommandToRR(
    const tesseract::environment::ChangeCollisionMarginsCommand::ConstPtr& cmd);

tesseract::environment::ChangeCollisionMarginsCommand::Ptr ChangeCollisionMarginsCommandFromRR(
    const rr_env_cmd::ChangeCollisionMarginsCommandPtr& cmd);

// ChangeJointAccelerationLimitsCommand
rr_env_cmd::ChangeJointAccelerationLimitsCommandPtr ChangeJointAccelerationLimitsCommandToRR(
    const tesseract::environment::ChangeJointAccelerationLimitsCommand::ConstPtr& cmd);

tesseract::environment::ChangeJointAccelerationLimitsCommand::Ptr ChangeJointAccelerationLimitsCommandFromRR(
    const rr_env_cmd::ChangeJointAccelerationLimitsCommandPtr& cmd);

// ChangeJointOriginCommand
rr_env_cmd::ChangeJointOriginCommandPtr ChangeJointOriginCommandToRR(
    const tesseract::environment::ChangeJointOriginCommand::ConstPtr& cmd);

tesseract::environment::ChangeJointOriginCommand::Ptr ChangeJointOriginCommandFromRR(
    const rr_env_cmd::ChangeJointOriginCommandPtr& cmd);

// ChangeJointPositionLimitsCommand
rr_env_cmd::ChangeJointPositionLimitsCommandPtr ChangeJointPositionLimitsCommandToRR(
    const tesseract::environment::ChangeJointPositionLimitsCommand::ConstPtr& cmd);

tesseract::environment::ChangeJointPositionLimitsCommand::Ptr ChangeJointPositionLimitsCommandFromRR(
    const rr_env_cmd::ChangeJointPositionLimitsCommandPtr& cmd);

// ChangeJointVelocityLimitsCommand
rr_env_cmd::ChangeJointVelocityLimitsCommandPtr ChangeJointVelocityLimitsCommandToRR(
    const tesseract::environment::ChangeJointVelocityLimitsCommand::ConstPtr& cmd);

tesseract::environment::ChangeJointVelocityLimitsCommand::Ptr ChangeJointVelocityLimitsCommandFromRR(
    const rr_env_cmd::ChangeJointVelocityLimitsCommandPtr& cmd);

// ChangeLinkCollisionEnabledCommand
rr_env_cmd::ChangeLinkCollisionEnabledCommandPtr ChangeLinkCollisionEnabledCommandToRR(
    const tesseract::environment::ChangeLinkCollisionEnabledCommand::ConstPtr& cmd);

tesseract::environment::ChangeLinkCollisionEnabledCommand::Ptr ChangeLinkCollisionEnabledCommandFromRR(
    const rr_env_cmd::ChangeLinkCollisionEnabledCommandPtr& cmd);

// ChangeLinkOriginCommand
rr_env_cmd::ChangeLinkOriginCommandPtr ChangeLinkOriginCommandToRR(
    const tesseract::environment::ChangeLinkOriginCommand::ConstPtr& cmd);

tesseract::environment::ChangeLinkOriginCommand::Ptr ChangeLinkOriginCommandFromRR(
    const rr_env_cmd::ChangeLinkOriginCommandPtr& cmd);

// ChangeLinkVisibilityCommand
rr_env_cmd::ChangeLinkVisibilityCommandPtr ChangeLinkVisibilityCommandToRR(
    const tesseract::environment::ChangeLinkVisibilityCommand::ConstPtr& cmd);

tesseract::environment::ChangeLinkVisibilityCommand::Ptr ChangeLinkVisibilityCommandFromRR(
    const rr_env_cmd::ChangeLinkVisibilityCommandPtr& cmd);

// ModifyAllowedCollisionsCommand
rr_env_cmd::ModifyAllowedCollisionsCommandPtr ModifyAllowedCollisionsCommandToRR(
    const tesseract::environment::ModifyAllowedCollisionsCommand::ConstPtr& cmd);

tesseract::environment::ModifyAllowedCollisionsCommand::Ptr ModifyAllowedCollisionsCommandFromRR(
    const rr_env_cmd::ModifyAllowedCollisionsCommandPtr& cmd);

// MoveJointCommand
rr_env_cmd::MoveJointCommandPtr MoveJointCommandToRR(const tesseract::environment::MoveJointCommand::ConstPtr& cmd);

tesseract::environment::MoveJointCommand::Ptr MoveJointCommandFromRR(const rr_env_cmd::MoveJointCommandPtr& cmd);

// MoveLinkCommand
rr_env_cmd::MoveLinkCommandPtr MoveLinkCommandToRR(const tesseract::environment::MoveLinkCommand::ConstPtr& cmd);

tesseract::environment::MoveLinkCommand::Ptr MoveLinkCommandFromRR(const rr_env_cmd::MoveLinkCommandPtr& cmd);

// RemoveAllowedCollisionLinkCommand
rr_env_cmd::RemoveAllowedCollisionLinkCommandPtr RemoveAllowedCollisionLinkCommandToRR(
    const tesseract::environment::RemoveAllowedCollisionLinkCommand::ConstPtr& cmd);

tesseract::environment::RemoveAllowedCollisionLinkCommand::Ptr RemoveAllowedCollisionLinkCommandFromRR(
    const rr_env_cmd::RemoveAllowedCollisionLinkCommandPtr& cmd);

// RemoveJointCommand
rr_env_cmd::RemoveJointCommandPtr RemoveJointCommandToRR(
    const tesseract::environment::RemoveJointCommand::ConstPtr& cmd);

tesseract::environment::RemoveJointCommand::Ptr RemoveJointCommandFromRR(const rr_env_cmd::RemoveJointCommandPtr& cmd);

// RemoveLinkCommand
rr_env_cmd::RemoveLinkCommandPtr RemoveLinkCommandToRR(const tesseract::environment::RemoveLinkCommand::ConstPtr& cmd);

tesseract::environment::RemoveLinkCommand::Ptr RemoveLinkCommandFromRR(const rr_env_cmd::RemoveLinkCommandPtr& cmd);

// ReplaceJointCommand
rr_env_cmd::ReplaceJointCommandPtr ReplaceJointCommandToRR(
    const tesseract::environment::ReplaceJointCommand::ConstPtr& cmd);

tesseract::environment::ReplaceJointCommand::Ptr ReplaceJointCommandFromRR(
    const rr_env_cmd::ReplaceJointCommandPtr& cmd);

// SetActiveContinuousContactManagerCommand
rr_env_cmd::SetActiveContinuousContactManagerCommandPtr SetActiveContinuousContactManagerCommandToRR(
    const tesseract::environment::SetActiveContinuousContactManagerCommand::ConstPtr& cmd);

tesseract::environment::SetActiveContinuousContactManagerCommand::Ptr SetActiveContinuousContactManagerCommandFromRR(
    const rr_env_cmd::SetActiveContinuousContactManagerCommandPtr& cmd);

// SetActiveDiscreteContactManagerCommand
rr_env_cmd::SetActiveDiscreteContactManagerCommandPtr SetActiveDiscreteContactManagerCommandToRR(
    const tesseract::environment::SetActiveDiscreteContactManagerCommand::ConstPtr& cmd);

tesseract::environment::SetActiveDiscreteContactManagerCommand::Ptr SetActiveDiscreteContactManagerCommandFromRR(
    const rr_env_cmd::SetActiveDiscreteContactManagerCommandPtr& cmd);

// Command
RobotRaconteur::RRValuePtr CommandToRR(const tesseract::environment::Command::ConstPtr& cmd);

tesseract::environment::Command::Ptr CommandFromRR(const RobotRaconteur::RRValuePtr& cmd);

// Commands
rr_env_cmd::CommandsPtr CommandsToRR(const tesseract::environment::Commands& commands);

tesseract::environment::Commands CommandsFromRR(const rr_env_cmd::CommandsPtr& commands);

} // namespace environment_conv
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_COMMANDS_CONV_H
