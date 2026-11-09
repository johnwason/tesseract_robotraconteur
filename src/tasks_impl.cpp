/**
 * @file tasks_impl.cpp
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

#include "tesseract_robotraconteur/tasks_impl.h"
#include "tesseract_robotraconteur/conv/tasks_conv.h"
#include <tesseract/task_composer/task_composer_server.h>
#include <tesseract/common/profile_dictionary.h>
#include <tesseract/command_language/constants.h>

#include "tesseract_robotraconteur/task_exec_gen_impl.h"
#include "tesseract_robotraconteur/tesseract_robotics_impl.h"

#include <tesseract/motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_profile.h>
#include <tesseract/motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_profile.h>

#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>
#include <tesseract/motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract/motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_move_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_osqp_solver_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_default_move_profile.h>
#include <tesseract/motion_planners/trajopt_ifopt/profile/trajopt_ifopt_osqp_solver_profile.h>
#include <tesseract/task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract/task_composer/planning/profiles/min_length_profile.h>
#include <tesseract/time_parameterization/isp/iterative_spline_parameterization_profiles.h>

using tesseract::command_language::InstructionPoly;
using tesseract::common::Serialization;
using tesseract::task_composer::TaskComposerDataStorage;

static const std::string TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask";
static const std::string TRAJOPT_IFOPT_DEFAULT_NAMESPACE = "TrajOptIfoptMotionPlannerTask";
static const std::string OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask";
static const std::string DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask";
static const std::string SIMPLE_DEFAULT_NAMESPACE = "SimpleMotionPlannerTask";
static const std::string MLT_DEFAULT_NAMESPACE = "MinLengthTask";
static const std::string ISP_DEFAULT_NAMESPACE = "IterativeSplineParameterizationTask";
static const std::string DCC_DEFAULT_NAMESPACE = "DiscreteContactCheckTask";

namespace RR = RobotRaconteur;

namespace tesseract_robotraconteur
{
TaskExecutorImpl::TaskExecutorImpl(tesseract::task_composer::TaskComposerServer::Ptr tesseract_server,
                                   const std::string& executor_name, RR_SHARED_PTR<TesseractRoboticsImpl> parent)
    : tesseract_server(tesseract_server), executor_name(executor_name), parent_(parent),
      profiles_(std::make_shared<tesseract::common::ProfileDictionary>())
{
    loadDefaultPlannerProfiles();
}

void TaskExecutorImpl::Init()
{
    // Nothing to do here for now
}

int32_t TaskExecutorImpl::get_worker_count() { return tesseract_server->getWorkerCount(executor_name); }

int32_t TaskExecutorImpl::get_task_count() { return tesseract_server->getTaskCount(executor_name); }

RR::GeneratorPtr<rr_tasks::TaskExecutorStatusPtr, void> TaskExecutorImpl::run(
    const rr_tasks::TaskExecutorInputPtr& input)
{
    RR_NULL_CHECK(input);
    auto task_name = input->task_name;
    RR_SHARED_PTR<TesseractRoboticsImpl> parent = parent_.lock();
    if (!parent)
    {
        throw RR::InvalidOperationException("Parent has been released");
    }
    tesseract::task_composer::TaskComposerDataStorage::Ptr data_storage =
        conv::TaskComposerDataStorageFromRR(input->data_storage, parent);
    if (!data_storage)
    {
        data_storage = std::make_shared<tesseract::task_composer::TaskComposerDataStorage>();
    }

    data_storage->setData("profiles", profiles_);

    auto& node = tesseract_server->getTask(task_name);

    return RR_MAKE_SHARED<TaskExecGenImpl>(tesseract_server, executor_name, node, data_storage);
}

// copied from tesseract_ros2
void TaskExecutorImpl::loadDefaultPlannerProfiles()
{
    // Add TrajOpt Default Profiles
    profiles_->addProfile(TRAJOPT_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::motion_planners::TrajOptDefaultMoveProfile>());
    profiles_->addProfile(TRAJOPT_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::motion_planners::TrajOptDefaultCompositeProfile>());
    profiles_->addProfile(TRAJOPT_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::motion_planners::TrajOptOSQPSolverProfile>());

    // Add TrajOpt IFOPT Default Profiles
    profiles_->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::motion_planners::TrajOptIfoptDefaultMoveProfile>());
    profiles_->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::motion_planners::TrajOptIfoptDefaultCompositeProfile>());
    profiles_->addProfile(TRAJOPT_IFOPT_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::motion_planners::TrajOptIfoptOSQPSolverProfile>());

    // Add Descartes Default Profiles
    profiles_->addProfile(DESCARTES_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::motion_planners::DescartesDefaultMoveProfile<double>>());

    // Add OMPL Default Profiles
    profiles_->addProfile(OMPL_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::motion_planners::OMPLRealVectorMoveProfile>());

    // Add Simple Default Profiles
    profiles_->addProfile(SIMPLE_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::motion_planners::SimplePlannerLVSNoIKMoveProfile>());

    // MinLengthTask calls the SimpleMotionPlanner to generate a seed path with waypoints interpolated in joint space
    profiles_->addProfile(MLT_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::task_composer::MinLengthProfile>());

    // Post hoc collision checking
    profiles_->addProfile(DCC_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
                          std::make_shared<tesseract::task_composer::ContactCheckProfile>());

    // Time parameterization
    profiles_->addProfile(
        ISP_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract::time_parameterization::IterativeSplineParameterizationCompositeProfile>());
    profiles_->addProfile(
        ISP_DEFAULT_NAMESPACE, tesseract::command_language::DEFAULT_PROFILE_KEY,
        std::make_shared<tesseract::time_parameterization::IterativeSplineParameterizationMoveProfile>());
}
} // namespace tesseract_robotraconteur
