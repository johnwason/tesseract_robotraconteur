/**
 * @file tasks_conv.cpp
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

#include "tesseract_robotraconteur/conv/tasks_conv.h"
#include "tesseract_robotraconteur/conv/common_conv.h"
#include "tesseract_robotraconteur/conv/command_language_conv.h"
#include "tesseract/task_composer/task_composer_keys.h"

#include "tesseract_robotraconteur/tesseract_robotics_impl.h"

#define RR_TASKS_PREFIX "experimental.tesseract_robotics.tasks"
#define RR_COMMAND_LANG_PREFIX "experimental.tesseract_robotics.command_language"
#define RR_ENV_PREFIX "experimental.tesseract_robotics.environment"

namespace RR = RobotRaconteur;

namespace tesseract_robotraconteur
{
namespace conv
{
RobotRaconteur::RRMapPtr<std::string, rr_tasks::TaskComposerKey> TaskComposerKeysToRR(
    const tesseract::task_composer::TaskComposerKeys& keys)
{
    auto ret_map = RR::AllocateEmptyRRMap<std::string, rr_tasks::TaskComposerKey>();
    const auto& data = keys.data();
    for (const auto& e : data)
    {
        rr_tasks::TaskComposerKeyPtr k(new rr_tasks::TaskComposerKey());
        k->port = e.first;
        switch (e.second.index())
        {
        case 0: {
            auto v1 = std::get<std::string>(e.second);
            k->keys = RR::stringToRRArray(v1);
            break;
        }
        case 1: {
            auto v1 = std::get<std::vector<std::string>>(e.second);
            k->keys = RR::stringVectorToRRList(v1);
            break;
            break;
        }
        default:
            throw RR::InternalErrorException("Internal tesseract error");
        }
        ret_map->insert(std::make_pair(e.first, k));
    }
    return ret_map;
}

tesseract::common::AnyPoly TaskPolyFromRR(const RR::RRValuePtr& data,
                                          const RR_SHARED_PTR<TesseractRoboticsImpl>& server)
{
    if (!data)
    {
        return tesseract::common::AnyPoly();
    }
    RR_NULL_CHECK(data);
    auto data_type = data->RRType();
    if (data_type == RR_COMMAND_LANG_PREFIX "."
                                            "CompositeInstruction")
    {
        auto instr = RR::rr_cast<rr_command::CompositeInstruction>(data);
        return tesseract::common::AnyPoly(conv::CompositeInstructionFromRR(instr));
    }
    else if (data_type == RR_ENV_PREFIX "."
                                        "EnvironmentHandle")
    {
        auto env_handle = RR::rr_cast<rr_env::EnvironmentHandle>(data);
        std::shared_ptr<const tesseract::environment::Environment> env =
            server->GetEnvironmentImpl(env_handle->name)->Environment();
        return tesseract::common::AnyPoly(env);
    }
    else
    {
        throw RR::InvalidArgumentException("Unknown task poly type");
    }
}

RR::RRValuePtr TaskPolyToRR(const tesseract::common::AnyPoly& data)
{
    if (data.isNull())
    {
        return nullptr;
    }
    auto data_type_id = data.getType();

    if (data_type_id == typeid(tesseract::command_language::CompositeInstruction))
    {
        auto instr = data.as<tesseract::command_language::CompositeInstruction>();
        return conv::CompositeInstructionToRR(instr);
    }
    else
    {
        // TODO: unknown data storage types
        // throw std::runtime_error("Unknown task poly type");
        return nullptr;
    }
}

RR::RRMapPtr<std::string, RR::RRValue> TaskComposerDataStorageToRR(
    const tesseract::task_composer::TaskComposerDataStorage::Ptr& data_storage)
{
    auto ret = RR::AllocateEmptyRRMap<std::string, RR::RRValue>();
    for (const auto& pair : data_storage->getData())
    {
        ret->insert(std::make_pair(pair.first, TaskPolyToRR(pair.second)));
    }
    return ret;
}

tesseract::task_composer::TaskComposerDataStorage::Ptr TaskComposerDataStorageFromRR(
    const RR::RRMapPtr<std::string, RR::RRValue>& data_storage, const RR_SHARED_PTR<TesseractRoboticsImpl>& server)
{
    if (!data_storage)
    {
        return nullptr;
    }
    auto ret = std::make_shared<tesseract::task_composer::TaskComposerDataStorage>();
    for (const auto& pair : *data_storage)
    {
        ret->setData(pair.first, TaskPolyFromRR(pair.second, server));
    }
    return ret;
}
} // namespace conv
} // namespace tesseract_robotraconteur
