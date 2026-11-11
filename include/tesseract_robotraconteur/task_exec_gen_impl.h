/**
 * @file task_exec_gen_impl.h
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

#ifndef TASK_EXEC_GEN_IMPL_H
#define TASK_EXEC_GEN_IMPL_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <RobotRaconteur.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <RobotRaconteurCompanion/Util/TaskGenerator.h>

#include <tesseract/task_composer/task_composer_server.h>
#include <tesseract/task_composer/task_composer_node.h>

#include "robotraconteur_generated.h"

namespace rr_tasks = experimental::tesseract_robotics::tasks;

namespace tesseract_robotraconteur
{

class TaskExecGenImpl : public virtual RobotRaconteur::Companion::Util::AsyncTaskGenerator<rr_tasks::TaskExecutorStatus>
{
  public:
    TaskExecGenImpl(tesseract::task_composer::TaskComposerServer::Ptr tesseract_server,
                    const std::string& executor_name, const tesseract::task_composer::TaskComposerNode& node,
                    tesseract::task_composer::TaskComposerDataStorage::Ptr data_storage);

    void Init();

    void StartTask() override;

    virtual void CloseRequested() override;

    virtual void AbortRequested() override;

  protected:
    void AbortTask();

    tesseract::task_composer::TaskComposerServer::Ptr tesseract_server;
    std::string executor_name;
    const tesseract::task_composer::TaskComposerNode& node;
    tesseract::task_composer::TaskComposerDataStorage::Ptr data_storage;
    std::weak_ptr<tesseract::task_composer::TaskComposerFuture> weak_fut;
};
} // namespace tesseract_robotraconteur

#endif // TASK_EXEC_GEN_IMPL_H
