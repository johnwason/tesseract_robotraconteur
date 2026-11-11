/**
 * @file tesseract_robotics_impl.cpp
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

#include "tesseract_robotraconteur/tesseract_robotics_impl.h"

#include <tesseract/task_composer/task_composer_server.h>
#include <tesseract/task_composer/task_composer_node.h>

#include "tesseract_robotraconteur/conv/tasks_conv.h"

namespace RR = RobotRaconteur;

namespace tesseract_robotraconteur
{
TesseractRoboticsImpl::TesseractRoboticsImpl(
    tesseract::environment::Environment::Ptr default_env,
    std::shared_ptr<tesseract::task_composer::TaskComposerServer> tesseract_server)
    : default_env(default_env), tesseract_server(tesseract_server)
{}

void TesseractRoboticsImpl::Init() {}

com::robotraconteur::device::DeviceInfoPtr TesseractRoboticsImpl::get_device_info() { return nullptr; }

RR::RRMapPtr<std::string, rr_env::EnvironmentInfo> TesseractRoboticsImpl::get_environments_info()
{
    auto ret = RR::AllocateEmptyRRMap<std::string, rr_env::EnvironmentInfo>();
    boost::mutex::scoped_lock lock(this_lock);
    for (const auto& pair : environments)
    {
        rr_env::EnvironmentInfoPtr info(new rr_env::EnvironmentInfo());
        info->name = pair.first;
        ret->insert(std::make_pair(pair.first, info));
    }
    return ret;
}

RR::RRMapPtr<std::string, rr_tasks::TaskComposerTaskInfo> TesseractRoboticsImpl::get_tasks_info()
{
    boost::mutex::scoped_lock lock(this_lock);
    auto ret = RR::AllocateEmptyRRMap<std::string, rr_tasks::TaskComposerTaskInfo>();
    std::vector<std::string> task_names = tesseract_server->getAvailableTasks();
    for (const auto& task_name : task_names)
    {
        auto& task_node = tesseract_server->getTask(task_name);
        rr_tasks::TaskComposerTaskInfoPtr ret1(new rr_tasks::TaskComposerTaskInfo());
        ret1->name = task_node.getName();
        ret1->uuid = task_node.getUUIDString();
        ret1->input_keys = conv::TaskComposerKeysToRR(task_node.getInputKeys());
        ret1->output_keys = conv::TaskComposerKeysToRR(task_node.getOutputKeys());
        ret->insert(std::make_pair(ret1->name, ret1));
    }
    return ret;
}

RR::RRMapPtr<std::string, rr_tasks::TaskExecutorInfo> TesseractRoboticsImpl::get_task_executors_info()
{
    auto ret = RR::AllocateEmptyRRMap<std::string, rr_tasks::TaskExecutorInfo>();
    auto exec_names = tesseract_server->getAvailableExecutors();
    boost::mutex::scoped_lock lock(this_lock);
    for (const auto& exec_name : exec_names)
    {
        rr_tasks::TaskExecutorInfoPtr info(new rr_tasks::TaskExecutorInfo());
        info->name = exec_name;
        ret->insert(std::make_pair(exec_name, info));
    }
    return ret;
}

std::string TesseractRoboticsImpl::load_environment(const std::string& environment_resource_name,
                                                    const std::string& new_environment_name)
{
    boost::mutex::scoped_lock lock(this_lock);
    if (environment_resource_name != "default")
    {
        throw RR::InvalidArgumentException("Only default environment is supported");
    }
    tesseract::environment::Environment::Ptr new_env = default_env->clone();
    new_env->setName(new_environment_name);
    auto rr_env = RR_MAKE_SHARED<EnvironmentImpl>(new_env, shared_from_this());
    rr_env->Init();
    environments.insert(std::make_pair(new_environment_name, rr_env));
    return new_environment_name;
}

std::string TesseractRoboticsImpl::clone_environment(const std::string& environment_name,
                                                     const std::string& new_environment_name)
{
    boost::mutex::scoped_lock lock(this_lock);
    auto env = environments.find(environment_name);
    if (env == environments.end())
    {
        throw RR::InvalidArgumentException("Environment not found");
    }
    tesseract::environment::Environment::Ptr new_env = env->second->Environment()->clone();
    new_env->setName(new_environment_name);
    auto rr_env = RR_MAKE_SHARED<EnvironmentImpl>(new_env, shared_from_this());
    rr_env->Init();
    environments.insert(std::make_pair(new_environment_name, rr_env));
    return new_environment_name;
}

rr_env::EnvironmentPtr TesseractRoboticsImpl::get_environments(const std::string& ind)
{
    return GetEnvironmentImpl(ind);
}

rr_tasks::TaskExecutorPtr TesseractRoboticsImpl::get_task_executors(const std::string& ind)
{
    boost::mutex::scoped_lock lock(this_lock);
    if (!tesseract_server->hasExecutor(ind))
    {
        throw RobotRaconteur::InvalidArgumentException("Invalid executor requested");
    }
    return RR_MAKE_SHARED<TaskExecutorImpl>(tesseract_server, ind, shared_from_this());
}

boost::shared_ptr<EnvironmentImpl> TesseractRoboticsImpl::GetEnvironmentImpl(const std::string& name)
{
    boost::mutex::scoped_lock lock(this_lock);
    auto env = environments.find(name);
    if (env == environments.end())
    {
        throw RR::InvalidArgumentException("Environment not found");
    }
    return env->second;
}

} // namespace tesseract_robotraconteur
