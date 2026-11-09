/**
 * @file tesseract_robotics_service.cpp
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

#include <iostream>

#include <boost/program_options.hpp>
#include "tesseract_robotraconteur/tesseract_robotics_impl.h"
#include <tesseract/task_composer/task_composer_server.h>
#include <drekar_launch_process_cpp/drekar_launch_process_cpp.h>
#include <tesseract/common/resource_locator.h>

#include "robotraconteur_generated.h"

using namespace RobotRaconteur;
using namespace tesseract_robotraconteur;

namespace po = boost::program_options;

int main(int argc, char** argv)
{
    try
    {
        std::string urdf_fname;
        std::string srdf_fname;
        std::string task_plugin_config_file;

        po::options_description generic_("Allowed options");
        generic_.add_options()("help,h", "print usage message")("urdf-file", po::value(&urdf_fname), "URDF filename")(
            "srdf-file", po::value(&srdf_fname), "SRDF filename")(
            "task-plugin-config-file", po::value(&task_plugin_config_file), "Task composer plugin configuration file");

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(generic_).run(), vm);
        po::notify(vm);

        // RobotRaconteur::Companion::RegisterStdRobDefServiceTypes();

        auto resource_locator = std::make_shared<tesseract::common::GeneralResourceLocator>();

        std::filesystem::path urdf_path(urdf_fname);
        std::filesystem::path srdf_path(srdf_fname);
        std::filesystem::path task_plugin_config_file_path(task_plugin_config_file);

        if (!std::filesystem::exists(urdf_path))
        {
            std::cerr << "URDF file does not exist: " << urdf_fname << std::endl;
            return 1;
        }

        if (!std::filesystem::exists(srdf_path))
        {
            std::cerr << "SRDF file does not exist: " << srdf_fname << std::endl;
            return 1;
        }

        if (!std::filesystem::exists(task_plugin_config_file_path))
        {
            std::cerr << "Task plugin config file does not exist: " << task_plugin_config_file << std::endl;
            return 1;
        }

        auto env = std::make_shared<tesseract::environment::Environment>();
        env->init(urdf_path, srdf_path, resource_locator);
        if (!env->isInitialized())
        {
            std::cerr << "Failed to initialize environment." << std::endl;
            return 1;
        }

        auto tesseract_server = std::make_shared<tesseract::task_composer::TaskComposerServer>();
        tesseract_server->loadConfig(task_plugin_config_file_path, *resource_locator);

        auto obj = RR_MAKE_SHARED<TesseractRoboticsImpl>(env, tesseract_server);
        obj->Init();

        RobotRaconteurNodeSetup node_setup(
            RobotRaconteurNode::sp(), ROBOTRACONTEUR_SERVICE_TYPES, "org.swri.tesseract", 63158,
            RobotRaconteurNodeSetupFlags_SERVER_DEFAULT | RobotRaconteurNodeSetupFlags_JUMBO_MESSAGE,
            RobotRaconteurNodeSetupFlags_SERVER_DEFAULT_ALLOWED_OVERRIDE, argc, argv);

        auto context = RobotRaconteurNode::s()->RegisterService("tesseract", "experimental.tesseract_robotics", obj);
        context->AddExtraImport("experimental.tesseract_robotics.command_language");

        std::cout << "Press Ctrl-C to quit" << std::endl;
        drekar_launch_process_cpp::CWaitForExit wait_exit;
        wait_exit.WaitForExit();
    }
    catch (std::exception& exp)
    {
        std::cerr << "Error: " << exp.what() << std::endl;
        return 1;
    }

    return 0;
}
