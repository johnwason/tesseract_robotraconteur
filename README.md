<p align="center"><img src="docs/figures/RRheader2.jpg"></p>

# Tesseract Robot Raconteur Service

The `tesseract_robotraconteur_service` provides a Tesseract planner in the form of a
[Robot Raconteur](https://github.com/robotraconteur/robotraconteur) service. The service is a wrapper around the
[Tesseract](https://github.com/tesseract-planning/tesseract) motion planning library. The service
provides a simple interface to the Tesseract motion planning library.

Robot Raconteur is a powerful open-source object-oriented communication framework for robotics, automation, and
other distributed systems. Robot Raconteur provides unique features such as an augmented-object
oriented data model, enhanced value types, true plug-and-play operation, and
advanced discovery/subscriptions. Robot Raconteur supports a large number of programming languages
including C++, Python, Java, C#, JavaScript, MATLAB and LabView. Supported platforms
include Windows, Linux, Mac OS X, iOS, Android, FreeBSD, and embedded systems.

The design of Robot Raconteur allows for the interface to closely match the underlying C++ interface.
This allows for the client to have a similar experience as if they were using the C++ interface. Because
of the large number of supported languages and platforms, this allows for a large number of clients to
be developed. See the service definitions specific to Tesseract in the `robdef` directory. Also
see the [Standard Robot Raconteur Service Definitions](https://github.com/robotraconteur/robotraconteur_standard_robdef)
which are available for use in any Robot Raconteur service.

Currently the service is capable of loading environments and executing planning operations.
Over time as much of the API as possible will be exposed.

## Running the Service

The service can be run as a local executable or using Docker. It is recommended that Docker be used if possible.

### Running the service using Docker

The service is available on [Docker Hub](https://hub.docker.com/r/wasontech/tesseract_robotraconteur).

Run the service using the following command:

```bash
sudo docker run --rm --net=host \
  -v /var/run/robotraconteur:/var/run/robotraconteur \
  -v /var/lib/robotraconteur:/var/lib/robotraconteur \
  -v /ws/install/share:/res \
  -e TESSERACT_RESOURCE_PATH=/res \
  -e URDF_FILE=/res/tesseract_support/urdf/abb_irb2400.urdf \
  -e SRDF_FILE=/res/tesseract_support/urdf/abb_irb2400.srdf \
  docker.io/wasontech/tesseract_robotraconteur
```

This relatively complex command is required to allow the service to run in the Docker container and
still be able to find the Robot Raconteur transport and discovery services. With
`--net=host` the service will use the host network stack. This is required for auto-discovery to work.


The following environment variables are available:

* `TESSERACT_RESOURCE_PATH`: This is the path for the resource loader to search for packages.
  Paths are separated with `:` (Unix) or `;` (Windows). The resource files will need to be
  mounted into the container. In this example, the resource directory `/ws/install/share` is
 mounted to `/res` in the container, and the environment variable is set to `/res`.
* `URDF_FILE`: The path to the URDF file to load. This is relative to the filesystem in the container.
* `SRDF_FILE`: The path to the SRDF file to load. This is relative to the filesystem in the container.
* `TASK_PLUGIN_CONFIG_FILE`: The path to the task plugin config file to load. For
  most cases this is not required and the default will be used. This is relative to the filesystem in the container.

### Building and Running the Service locally

Building Tesseract can be difficult. See
the [tesseract_planning](https://github.com/tesseract-robotics/tesseract_planning) repository for more
information on building Tesseract. Use the `dependencies.repos` file in this repository to get the
additional dependencies required to build the service.

Once built, use the following command to run the service:

```bash
export TESSERACT_RESOURCE_PATH=/ws/install/share
./tesseract_robotraconteur_service --urdf-file=/ws/install/share/tesseract/support/urdf/abb_irb2400.urdf \
     --srdf-file=/ws/install/share/tesseract/support/urdf/abb_irb2400.srdf \
     --task-plugin-config-file=/ws/install/share/tesseract_planning/task_composer/config/task_composer_plugins.yaml
```

The following options are available:

* `--urdf-file`: The path to the URDF file to load
* `--srdf-file`: The path to the SRDF file to load
* `--task-plugin-config-file`: The path to the task plugin config file to load
* `export TESSERACT_RESOURCE_PATH`: This is the path for the resource loader to search for packages.
  Paths are separated with `:` (Unix) or `;` (Windows). This is required for the service to find things
  like meshes and configuration files.

The standard Robot Raconteur options are also available. See
[Robot Raconteur Node Command Line Options](https://github.com/robotraconteur/robotraconteur/wiki/Command-Line-Options)
for more information.

## Using the Service

The service is a wrapper around the Tesseract motion planning library. The following example
shows how to use the service in Python. The same example is available in C++. See the
`examples` directory for more examples.

```python
# Robot Raconteur Tesseract Service Example: Freespace Move Example
#
# The client example only requires the robotraconteur and companion library.
# The Tesseract libraries are not required for the client.
#
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
    command_lang_const = RRN.GetConstants("experimental.tesseract_robotics.command_language",c)
    action_const = RRN.GetConstants("com.robotraconteur.action",c)
    cart_wp_type = RRN.GetStructureType("experimental.tesseract_robotics.command_language.CartesianWaypoint",c)
    move_instr_type = RRN.GetStructureType("experimental.tesseract_robotics.command_language.MoveInstruction",c)
    composite_instr_type = RRN.GetStructureType("experimental.tesseract_robotics.command_language.CompositeInstruction",c)
    manip_info_type = RRN.GetStructureType("experimental.tesseract_robotics.common.ManipulatorInfo",c)
    task_exec_input_type = RRN.GetStructureType("experimental.tesseract_robotics.tasks.TaskExecutorInput",c)
    env_handle_type = RRN.GetStructureType("experimental.tesseract_robotics.environment.EnvironmentHandle",c)

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
        wp.transform = geom.xyz_rpy_to_transform(xyz,rpy)

        # Create a move instruction and set the waypoint
        instr = move_instr_type()
        instr.waypoint = RR.VarValue(wp, "experimental.tesseract_robotics.command_language.CartesianWaypoint")
        instr.profile = "DEFAULT"
        instr.move_type = command_lang_const["MoveInstructionType"]["freespace"]
        instr.uuid = uuid_util.NewRandomUuid()
        return RR.VarValue(instr, "experimental.tesseract_robotics.command_language.MoveInstruction")

    # Create the instructions
    instr1 = build_move_instr([0.8, -0.3, 1.455], np.deg2rad([0,90,0]))
    instr2 = build_move_instr([0.8, 0.3, 1.455], np.deg2rad([0,90,0]))
    instr3 = build_move_instr([0.8, 0.5, 1.455], np.deg2rad([0,90,0]))

    # Create and configure the manipulator info structure
    manip_info = manip_info_type()
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    manip_info.manipulator = "manipulator"

    # Create and configure the composite instruction
    composite = composite_instr_type()
    composite.order = command_lang_const["CompositeInstructionOrder"]["ordered"]
    composite.instructions = [instr1, instr2] #, instr3]
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
```

## License

Apache 2.0

## Acknowledgment

This work was supported in part by the Advanced Robotics for Manufacturing ("ARM") Institute under Agreement Number W911NF-17-3-0004 sponsored by the Office of the Secretary of Defense. The views and conclusions contained in this document are those of the authors and should not be interpreted as representing the official policies, either expressed or implied, of either ARM or the Office of the Secretary of Defense of the U.S. Government. The U.S. Government is authorized to reproduce and distribute reprints for Government purposes, notwithstanding any copyright notation herein.

This work was supported in part by the New York State Empire State Development Division of Science, Technology and Innovation (NYSTAR) under contract C160142.

![](docs/figures/arm_logo.jpg) ![](docs/figures/nys_logo.jpg)
