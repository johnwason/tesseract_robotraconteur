/**
 * @file tasks_impl.h
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

#ifndef TESSERACT_ROBOTRACONTEUR_TASKS_IMPL_H
#define TESSERACT_ROBOTRACONTEUR_TASKS_IMPL_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
#include <RobotRaconteurCompanion/StdRobDef/StdRobDefAll.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "robotraconteur_generated.h"
#include <tesseract/task_composer/task_composer_server.h>

namespace rr_tasks = experimental::tesseract_robotics::tasks;

namespace tesseract_robotraconteur
{
class TesseractRoboticsImpl;
class TaskExecutorImpl : public virtual rr_tasks::TaskExecutor_default_impl
{
  public:
    TaskExecutorImpl(tesseract::task_composer::TaskComposerServer::Ptr tesseract_server,
                     const std::string& executor_name, RR_SHARED_PTR<TesseractRoboticsImpl> parent);

    void Init();

    int32_t get_worker_count() override;

    int32_t get_task_count() override;

    RobotRaconteur::GeneratorPtr<rr_tasks::TaskExecutorStatusPtr, void> run(
        const rr_tasks::TaskExecutorInputPtr& input) override;

  protected:
    tesseract::task_composer::TaskComposerServer::Ptr tesseract_server;
    std::string executor_name;
    RR_WEAK_PTR<TesseractRoboticsImpl> parent_;

    void loadDefaultPlannerProfiles();
    std::shared_ptr<tesseract::common::ProfileDictionary> profiles_;
};
} // namespace tesseract_robotraconteur

#endif // TESSERACT_ROBOTRACONTEUR_TASKS_IMPL_H
