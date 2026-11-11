// Robot Raconteur Tesseract Service Example: Freespace Move Example C++
//
// This is an example client program showing how to use the Tesseract
// service with Robot Raconteur. It expects the abb_irb2400 example
// scene from tesseract/support package to be loaded into the service.
//
// The service should be started prior to running this program. The service
// can be started using the docker image or from the command line.
// See the readme for instructions on how to start the service
// using the docker image. From the command line, use the following command.
// Adjust /ws/install/share/tesseract/support to the location of the tesseract/support.
//
//  export TESSERACT_RESOURCE_PATH=/ws/install/share
//  tesseract_robotraconteur_service --urdf-file=/ws/install/share/tesseract/support/urdf/abb_irb2400.urdf \
//     --srdf-file=/ws/install/share/tesseract/support/urdf/abb_irb2400.srdf \
//     --task-plugin-config-file=/ws/install/share/tesseract_planning/task_composer/config/task_composer_plugins.yaml
//
// Robot Raconteur with C++ requires the RobotRaconteurCore, RobotRaconteurCompanion, and the generated
// "thunk" source from the service definition "robdef" files that are specific to the Tesseract
// service. The CMake command ROBOTRACONTEUR_GENERATE_THUNK is used to generate
// the "thunk" source files automatically at compile time. Because C++ is statically typed,
// the service definitions must be available at compile time. This is unlike Python where
// the service definitions are loaded at runtime.
//
// See the Robot Raconteur documentation for more information on using Robot Raconteur with C++.

#include <iostream>
#include "robotraconteur_generated.h"
#include <boost/uuid/uuid_generators.hpp>
#include <RobotRaconteurCompanion/Converters/EigenConverters.h>

// Convenience namespace aliases
namespace RR = RobotRaconteur;

namespace rr_tasks = experimental::tesseract_robotics::tasks;
namespace RRC_Eigen = RobotRaconteur::Companion::Converters::Eigen;
namespace rr_tesseract = experimental::tesseract_robotics;
namespace rr_common = experimental::tesseract_robotics::common;
namespace rr_command = experimental::tesseract_robotics::command_language;

// TODO: Add this function to the Robot Raconteur companion library
com::robotraconteur::uuid::UUID new_random_uuid()
{
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    com::robotraconteur::uuid::UUID ret;
    memcpy(ret.s.uuid_bytes.data(), &uuid, sizeof(uuid));
    return ret;
}

// Helper function to build a move instruction from a waypoint
auto build_move_instr(const Eigen::Isometry3d& wp)
{
    // Create a Cartesian waypoint from the Eigen Isometry transform
    rr_command::CartesianWaypointPtr wp_ptr(new rr_command::CartesianWaypoint());
    // Use a companion helper function to convert the Eigen transform to a Robot Raconteur transform
    wp_ptr->transform = RRC_Eigen::ToTransform(wp);
    wp_ptr->lower_tolerance = RR::AllocateRRArray<double>(0);
    wp_ptr->upper_tolerance = RR::AllocateRRArray<double>(0);

    // Create a move instruction from the waypoint
    rr_command::MoveInstructionPtr move_instr(new rr_command::MoveInstruction());
    move_instr->waypoint = wp_ptr;
    move_instr->profile = "DEFAULT";
    move_instr->move_type = rr_command::MoveInstructionType::freespace;
    move_instr->uuid = new_random_uuid();
    return move_instr;
}

// Main entry point
int main(int argc, char** argv)
{

    // Initialize Robot Raconteur using ClientNodeSetup, the generated thunk source types, and command line arguments
    RR::ClientNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES, argc, argv);

    // Connect to the service. In this case, a localhost url is used. If the service were running
    // on a remote node, the URL would be "rr+tcp://hostname.example.com:port" for TCP. An IP
    // address can also be used directly.
    //
    // Robot Raconteur has advanced discovery and subscriptions that can be used instead of URLs. See
    // the documentation for more information.
    rr_tesseract::TesseractRoboticsPtr c = RR::rr_cast<rr_tesseract::TesseractRobotics>(
        RR::RobotRaconteurNode::s()->ConnectService("rr+tcp://localhost:63158?service=tesseract"));

    // Get the available task pipeline information from the service. Tesseract task pipelines
    // execute planning and other operations
    auto task_info = c->get_tasks_info();
    std::string output_key = RR::RRArrayToString(
        RR::rr_cast<RR::RRArray<char>>(task_info->at("FreespacePipeline")->output_keys->at("program")->keys));
    std::string input_key = RR::RRArrayToString(
        RR::rr_cast<RR::RRArray<char>>(task_info->at("FreespacePipeline")->input_keys->at("planning_input")->keys));

    // Create the environment. In this case, the environment is loaded from the service. The
    // available environments are specified when the service is started. "default" is the
    // default environment.
    c->load_environment("default", "env");

    // Create the EnvironmentHandle to pass to the planner
    rr_tesseract::environment::EnvironmentHandlePtr env_handle(new rr_tesseract::environment::EnvironmentHandle());
    env_handle->name = "env";

    // Create the instructions
    auto instr1 = build_move_instr(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, -0.3, 1.455) *
                                   Eigen::Quaterniond(0.70710678, 0, 0.70710678, 0));
    auto instr2 = build_move_instr(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.9, 0.3, 1.455) *
                                   Eigen::Quaterniond(0.70710678, 0, 0.70710678, 0));

    // Create and configure the manipulator info structure. The default tcp_offset is all zeros, which
    // is not a valid rotation, so set it to the identity.
    rr_common::ManipulatorInfoPtr manip_info(new rr_common::ManipulatorInfo());
    manip_info->manipulator = "manipulator";
    manip_info->tcp_frame = "tool0";
    manip_info->working_frame = "base_link";
    manip_info->tcp_offset = RRC_Eigen::ToTransform(Eigen::Isometry3d::Identity());

    // Create and configure the composite instruction
    rr_command::CompositeInstructionPtr composite(new rr_command::CompositeInstruction());
    composite->order = rr_command::CompositeInstructionOrder::ordered;
    composite->instructions = RR::AllocateEmptyRRList<RR::RRValue>();
    composite->instructions->push_back(instr1);
    composite->instructions->push_back(instr2);
    composite->profile = "DEFAULT";
    composite->manipulator_info = manip_info;
    composite->uuid = new_random_uuid();

    // Create and configure the planning problem
    rr_tasks::TaskExecutorInputPtr exec_input(new rr_tasks::TaskExecutorInput());
    exec_input->task_name = "FreespacePipeline";
    exec_input->data_storage = RR::AllocateEmptyRRMap<std::string, RR::RRValue>();
    exec_input->data_storage->insert(std::make_pair("environment", env_handle));
    exec_input->data_storage->insert(std::make_pair(input_key, composite));

    // Find first executor
    auto executors_info = c->get_task_executors_info();
    auto default_task_executor = executors_info->begin()->second->name;

    // Get the default task executor. In this case, the task executor will be using Taskflow
    // to execute the pipeline
    auto executor = c->get_task_executors(default_task_executor);

    // Run the pipeline
    auto exec_gen = executor->run(exec_input);

    // The pipeline returns a Robot Raconteur "Generator". Generators are a type of coroutine
    // that are used either for a long running operation or a sequence operation.
    // Repeat TryNext() until the generator is complete.
    rr_tasks::TaskExecutorStatusPtr status;
    bool res;
    do
    {
        res = exec_gen->TryNext(status);
    } while (res);

    // Check the status of the pipeline take make sure it completed successfully
    if (status->action_status != com::robotraconteur::action::ActionStatusCode::complete)
    {
        std::cerr << "Planning did not complete successfully" << std::endl;
        return 1;
    }

    // Retrieve the output plan from the data storage
    auto output = RR::rr_cast<rr_command::CompositeInstruction>(status->data_storage->at(output_key));

    // Extract the joint trajectory from the output plan
    std::vector<Eigen::VectorXd> joint_trajectory;
    for (auto instr : *output->instructions)
    {
        auto move_instr = RR::rr_cast<rr_command::MoveInstruction>(instr);
        auto wp = RR::rr_cast<rr_command::StateWaypoint>(move_instr->waypoint);
        joint_trajectory.push_back(RRC_Eigen::RRArrayToEigen<Eigen::VectorXd>(wp->position));
    }

    // Print out the joint trajectory
    for (auto& j : joint_trajectory)
    {
        std::cout << "[" << j.transpose() << "]" << std::endl;
    }

    // TODO: plot result in viewer!

    return 0;
}
