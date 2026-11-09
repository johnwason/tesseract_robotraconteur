/**
 * @file environment_impl.h
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

#ifndef TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_IMPL_H
#define TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_IMPL_H

#include <tesseract/environment/environment.h>

#include "robotraconteur_generated.h"

namespace rr_env = experimental::tesseract_robotics::environment;
namespace rr_sg = experimental::tesseract_robotics::scene_graph;

namespace tesseract_robotraconteur
{
class TesseractRoboticsImpl;
class EnvironmentImpl : public virtual rr_env::Environment_default_impl,
                        public std::enable_shared_from_this<EnvironmentImpl>
{
  public:
    EnvironmentImpl(tesseract::environment::Environment::Ptr env, RR_SHARED_PTR<TesseractRoboticsImpl> parent);

    void Init();

    std::string get_name() override;

    RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char>> get_joint_names() override;

    RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char>> get_link_names() override;

    rr_sg::JointPtr getf_joint(const std::string& joint_name) override;

    rr_sg::LinkPtr getf_link(const std::string& link_name) override;

    int32_t apply_command(const RobotRaconteur::RRValuePtr& command) override;

    int32_t apply_commands(const rr_env::commands::CommandsPtr& commands) override;

    rr_env::commands::CommandsPtr getf_command_history() override;

    tesseract::environment::Environment::Ptr Environment() const;

    RobotRaconteur::RRMapPtr<std::string, RobotRaconteur::RRNamedArray<com::robotraconteur::geometry::Transform>>
    getf_current_floating_joint_values(
        const RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char>>& joint_names) override;

    RobotRaconteur::RRArrayPtr<double> getf_current_joint_values(
        const RobotRaconteur::RRListPtr<RobotRaconteur::RRArray<char>>& joint_names) override;

    rr_sg::SceneStatePtr getf_state_all() override;

    void setf_state(const RobotRaconteur::RRMapPtr<std::string, RobotRaconteur::RRArray<double>>& joints,
                    const RobotRaconteur::RRMapPtr<
                        std::string, RobotRaconteur::RRNamedArray<com::robotraconteur::geometry::Transform>>&
                        floating_joints) override;

  protected:
    tesseract::environment::Environment::Ptr env_;
    RR_WEAK_PTR<TesseractRoboticsImpl> parent_;
};
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_ENVIRONMENT_IMPL_H
