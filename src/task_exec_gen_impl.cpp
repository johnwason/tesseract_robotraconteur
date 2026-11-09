/**
 * @file task_exec_gen_impl.cpp
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

#include "tesseract_robotraconteur/task_exec_gen_impl.h"

#include "tesseract_robotraconteur/conv/tasks_conv.h"

#include <tesseract/task_composer/task_composer_future.h>
#include <tesseract/task_composer/task_composer_context.h>

namespace rr_action = com::robotraconteur::action;
namespace RR = RobotRaconteur;

namespace tesseract_robotraconteur
{
TaskExecGenImpl::TaskExecGenImpl(tesseract::task_composer::TaskComposerServer::Ptr tesseract_server,
                                 const std::string& executor_name,
                                 const tesseract::task_composer::TaskComposerNode& node,
                                 tesseract::task_composer::TaskComposerDataStorage::Ptr data_storage)
    : AsyncTaskGenerator(RR::RobotRaconteurNode::sp(), 5000, -1), tesseract_server(tesseract_server),
      executor_name(executor_name), node(node), data_storage(data_storage)
{}

void TaskExecGenImpl::Init() {}

void TaskExecGenImpl::StartTask()
{
    auto fut1 = tesseract_server->run(node, data_storage, false, executor_name);
    std::shared_ptr<tesseract::task_composer::TaskComposerFuture> fut = std::move(fut1);
    weak_fut = fut;
    auto shared_this = RR_DYNAMIC_POINTER_CAST<TaskExecGenImpl>(shared_from_this());
    boost::thread([shared_this, fut]() {
        try
        {
            fut->wait();
            rr_tasks::TaskExecutorStatusPtr status(new rr_tasks::TaskExecutorStatus());
            if (fut->context->isAborted())
            {
                throw RR::OperationAbortedException("Task aborted");
            }
            else if (fut->context->isSuccessful())
            {
                status->action_status = rr_action::ActionStatusCode::complete;
            }
            else
            {
                status->action_status = rr_action::ActionStatusCode::failed;
            }
            status->data_storage = conv::TaskComposerDataStorageToRR(fut->context->data_storage);
            shared_this->SetResult(status);
            return;
        }
        catch (std::exception& e)
        {
            auto rr_exp = RR::RobotRaconteurExceptionUtil::ExceptionToSharedPtr(e);
            shared_this->SetResultException(rr_exp);
            return;
        }
    });
}

void TaskExecGenImpl::CloseRequested()
{
    tesseract::task_composer::TaskComposerFuture::Ptr fut = weak_fut.lock();
    if (!fut)
    {
        return;
    }

    // TODO: check if task is running
    fut->context->abort();
}

void TaskExecGenImpl::AbortRequested()
{
    tesseract::task_composer::TaskComposerFuture::Ptr fut = weak_fut.lock();
    if (!fut)
    {
        return;
    }

    // TODO: check if task is running
    fut->context->abort();
}

} // namespace tesseract_robotraconteur
