# Robot Raconteur Tesseract Service Example: Freespace Move Example
#
# This is an example client program showing how to use the Tesseract
# service with Robot Raconteur. It expects the abb_irb2400 example
# scene from tesseract/support package to be loaded into the service.
#
# The service should be started prior to running this program. The service
# can be started using the docker image or from the command line.
# See the readme for instructions on how to start the service
# using the docker image. From the command line, use the following command.
# Adjust /ws/install/share/tesseract/support to the location of the tesseract/support.
#
#  export TESSERACT_RESOURCE_PATH=/ws/install/share
#  tesseract_robotraconteur_service --urdf-file=/ws/install/share/tesseract/support/urdf/abb_irb2400.urdf \
#     --srdf-file=/ws/install/share/tesseract/support/urdf/abb_irb2400.srdf \
#     --task-plugin-config-file=/ws/install/share/tesseract_planning/task_composer/config/task_composer_plugins.yaml
#
# Once the service is running, the client can be run from another terminal. The robotraconteur and
# robotraconteurcomanion python packages must be installed. Install using the following commands:
#
#    python3 -m pip install robotraconteur robotraconteurcompanion
#
# The client only requires the robotraconteur and robotraconteurcompanion python packages. The tesseract
# package is not required on the client side.
#
# To run the client use the following command:
#
#    python3 freespace_move_example.py
#
# Note the use of RR.VarValue throughout the example. This is required to specify the type of the
# variant data. This is required because Python does not carry enough information to automatically
# determine the type of the variant data.

# Import Robot Raconteur and general Python modules. The companion package is not
# required but contains some utility classes for working with Robot Raconteur that
# simplify working with advanced types.
from RobotRaconteur.Client import *
from RobotRaconteurCompanion.Util.GeometryUtil import GeometryUtil
from RobotRaconteurCompanion.Util.UuidUtil import UuidUtil
import numpy as np


def main():

    # Connect to the service. In this case, a localhost url is used. If the service were running
    # on a remote node, the URL would be "rr+tcp://hostname.example.com:port" for TCP. An IP
    # address can also be used directly.
    #
    # Robot Raconteur has advanced discovery and subscriptions that can be used instead of URLs. See
    # the documentation for more information.
    c = RRN.ConnectService('rr+tcp://localhost:63158?service=tesseract')

    # Initialize the helper classes from the companion.
    geom = GeometryUtil(client_obj=c)
    uuid_util = UuidUtil(client_obj=c)

    # Retrieve the constants and structure types from the service. Robot Raconteur provides
    # plug-and-play capabilities, so this information is provided at runtime by the service.
    # The client does not need to be configured with this information, it is provided
    # automatically by the service.
    command_lang_const = RRN.GetConstants("experimental.tesseract_robotics.command_language", c)
    action_const = RRN.GetConstants("com.robotraconteur.action", c)
    cart_wp_type = RRN.GetStructureType("experimental.tesseract_robotics.command_language.CartesianWaypoint", c)
    move_instr_type = RRN.GetStructureType("experimental.tesseract_robotics.command_language.MoveInstruction", c)
    composite_instr_type = RRN.GetStructureType(
        "experimental.tesseract_robotics.command_language.CompositeInstruction", c)
    manip_info_type = RRN.GetStructureType("experimental.tesseract_robotics.common.ManipulatorInfo", c)
    task_exec_input_type = RRN.GetStructureType("experimental.tesseract_robotics.tasks.TaskExecutorInput", c)
    env_handle_type = RRN.GetStructureType("experimental.tesseract_robotics.environment.EnvironmentHandle", c)

    # Get the available task pipeline information from the service. Tesseract task pipelines
    # execute planning and other operations
    task_info = c.tasks_info
    output_key = task_info["FreespacePipeline"].output_keys["program"].keys.data
    input_key = task_info["FreespacePipeline"].input_keys["planning_input"].keys.data

    # Create the environment. In this case, the environment is loaded from the service. The
    # available environments are specified when the service is started. "default" is the
    # default environment.
    c.load_environment("default", "env")

    # Create the EnvironmentHandle to pass to the planner
    env_handle = env_handle_type()
    env_handle.name = "env"
    env_handle_input = RR.VarValue(env_handle, "experimental.tesseract_robotics.environment.EnvironmentHandle")

    # Helper function to make a Move instruction
    def build_move_instr(xyz, rpy):
        # Create a cartesian waypoint. Joint waypoints are also available.
        wp = cart_wp_type()
        wp.transform = geom.xyz_rpy_to_transform(xyz, rpy)

        # Create a move instruction and set the waypoint
        instr = move_instr_type()
        instr.waypoint = RR.VarValue(wp, "experimental.tesseract_robotics.command_language.CartesianWaypoint")
        instr.profile = "DEFAULT"
        instr.move_type = command_lang_const["MoveInstructionType"]["freespace"]
        instr.uuid = uuid_util.NewRandomUuid()
        return RR.VarValue(instr, "experimental.tesseract_robotics.command_language.MoveInstruction")

    # Create the instructions
    instr1 = build_move_instr([0.8, -0.3, 1.455], np.deg2rad([0, 90, 0]))
    instr2 = build_move_instr([0.8, 0.3, 1.455], np.deg2rad([0, 90, 0]))
    instr3 = build_move_instr([0.8, 0.5, 1.455], np.deg2rad([0, 90, 0]))

    # Create and configure the manipulator info structure
    manip_info = manip_info_type()
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    manip_info.manipulator = "manipulator"

    # Create and configure the composite instruction
    composite = composite_instr_type()
    composite.order = command_lang_const["CompositeInstructionOrder"]["ordered"]
    composite.instructions = [instr1, instr2]  # , instr3]
    composite.manipulator_info = manip_info
    composite.uuid = uuid_util.NewRandomUuid()
    composite_input = RR.VarValue(composite, "experimental.tesseract_robotics.command_language.CompositeInstruction")

    # Find first executor
    executors_info = c.task_executors_info
    default_task_executor = next(iter(executors_info.values())).name

    # Get the taskflow task executor
    executor = c.get_task_executors(default_task_executor)

    # Create the task executor input and set the pipeline name. In this case, we are using
    # the FreespacePipeline. Other pipelines are available. These pipelines are configured
    # when the service is started.
    exec_input = task_exec_input_type()
    exec_input.task_name = "FreespacePipeline"
    exec_input.data_storage = {}
    exec_input.data_storage["environment"] = env_handle_input
    exec_input.data_storage[input_key] = composite_input

    # Run the pipeline
    exec_gen = executor.run(exec_input)

    # The pipeline returns a Robot Raconteur "Generator". Generators are a type of coroutine
    # that are used either for a long running operation or a sequence operation.
    # Repeat TryNext() until the generator is complete.
    status = None
    while True:
        res, status1 = exec_gen.TryNext()
        if not res:
            break
        status = status1

    # Check the status of the pipeline take make sure it completed successfully
    assert status.action_status == action_const["ActionStatusCode"]["complete"]

    # Retrieve the output plan from the data storage
    output_plan = status.data_storage[output_key]

    # Extract and print the planned waypoints
    wp_positions = []

    for instr_v in output_plan.data.instructions:
        instr = instr_v.data
        wp = instr.waypoint.data
        wp_positions.append(wp.position)

    print(wp_positions)

    # TODO: plot result in viewer!


if __name__ == '__main__':
    main()
