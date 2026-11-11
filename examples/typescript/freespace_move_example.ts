// Robot Raconteur Tesseract Service Example: Freespace Move Example
//
// This is an example client program showing how to use the Tesseract service with Robot Raconteur
// from TypeScript. It is the same program as examples/python/freespace_move_example.py. It expects
// the abb_irb2400 example scene from the tesseract/support package to be loaded into the service.
//
// The service should be started prior to running this program. See the readme for instructions on
// how to start the service.
//
// The client requires Node.js 22.18 or newer, which runs .ts files directly. The robotraconteur and
// robotraconteur-node packages are not on npm yet. Clone and build them next to each other as
// described in the robotraconteur-node readme:
//
//    https://github.com/johnwason/robotraconteur_typescript_node#installing
//
// Then install them in this folder and run the client:
//
//    npm install --no-save /path/to/robotraconteur_typescript /path/to/robotraconteur_typescript_node
//    node freespace_move_example.ts [connection URL]
//
// Note the use of VarValue throughout the example. It specifies the type of varvalue data, since
// JavaScript objects do not carry their Robot Raconteur type.

import { randomUUID } from "node:crypto";
import { getConstants, newStructure, RobotRaconteurNode, VarValue } from "robotraconteur";
import { TcpTransport } from "robotraconteur-node";

const COMMAND_LANGUAGE = "experimental.tesseract_robotics.command_language";

interface TaskComposerKey {
    keys: VarValue;
}

interface TaskComposerTaskInfo {
    input_keys: Map<string, TaskComposerKey>;
    output_keys: Map<string, TaskComposerKey>;
}

interface TaskExecutorInfo {
    name: string;
}

interface TaskExecutorStatus {
    action_status: number;
    data_storage: Map<string, VarValue>;
}

interface CompositeInstruction {
    instructions: VarValue[];
}

function newRandomUuid() {
    return { uuid_bytes: Uint8Array.from(Buffer.from(randomUUID().replaceAll("-", ""), "hex")) };
}

function xyzRpyToTransform([x, y, z]: number[], [roll, pitch, yaw]: number[]) {
    const [cr, sr] = [Math.cos(roll / 2), Math.sin(roll / 2)];
    const [cp, sp] = [Math.cos(pitch / 2), Math.sin(pitch / 2)];
    const [cy, sy] = [Math.cos(yaw / 2), Math.sin(yaw / 2)];
    return {
        rotation: {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy
        },
        translation: { x, y, z }
    };
}

const deg2rad = (degrees: number) => (degrees * Math.PI) / 180;

// Connect to the service. In this case, a localhost url is used. If the service were running on a
// remote node, the URL would be "rr+tcp://hostname.example.com:port?service=tesseract".
//
// robotraconteur-node also provides discovery and subscriptions that can be used instead of URLs.
const url = process.argv[2] ?? "rr+tcp://localhost:63158?service=tesseract";

const node = new RobotRaconteurNode();
node.registerTransport(new TcpTransport(node));

try {
    const c = await node.connectService(url);

    // Get the enums from the service definitions. The definitions are pulled from the service when
    // connecting.
    const commandLanguageConst = getConstants(c, COMMAND_LANGUAGE);
    const actionConst = getConstants(c, "com.robotraconteur.action");

    // Helper function to make a Move instruction. newStructure creates a structure with every field
    // set to its default, so only the fields that matter need to be set.
    const buildMoveInstr = (xyz: number[], rpy: number[]): VarValue => {
        // Create a cartesian waypoint. Joint waypoints are also available.
        const wp = newStructure(c, `${COMMAND_LANGUAGE}.CartesianWaypoint`);
        wp.transform = xyzRpyToTransform(xyz, rpy);

        // Create a move instruction and set the waypoint
        const instr = newStructure(c, `${COMMAND_LANGUAGE}.MoveInstruction`);
        instr.uuid = newRandomUuid();
        instr.waypoint = new VarValue(wp, `${COMMAND_LANGUAGE}.CartesianWaypoint`);
        instr.profile = "DEFAULT";
        instr.move_type = commandLanguageConst.MoveInstructionType.freespace;
        return new VarValue(instr, `${COMMAND_LANGUAGE}.MoveInstruction`);
    };

    // Get the available task pipeline information from the service. Tesseract task pipelines
    // execute planning and other operations
    const tasksInfo = (await c.getProperty("tasks_info")) as Map<string, TaskComposerTaskInfo>;
    const freespace = tasksInfo.get("FreespacePipeline")!;
    const outputKey = freespace.output_keys.get("program")!.keys.value as string;
    const inputKey = freespace.input_keys.get("planning_input")!.keys.value as string;

    // Create the environment. In this case, the environment is loaded from the service. The
    // available environments are specified when the service is started. "default" is the
    // default environment.
    await c.callFunction("load_environment", ["default", "env"]);

    // Create the EnvironmentHandle to pass to the planner
    const envHandle = newStructure(c, "experimental.tesseract_robotics.environment.EnvironmentHandle");
    envHandle.name = "env";
    const envHandleInput = new VarValue(envHandle, "experimental.tesseract_robotics.environment.EnvironmentHandle");

    // Create the instructions
    const rpy = [0, deg2rad(90), 0];
    const instr1 = buildMoveInstr([0.8, -0.3, 1.455], rpy);
    const instr2 = buildMoveInstr([0.8, 0.3, 1.455], rpy);

    // Create and configure the manipulator info structure. The default tcp_offset is all zeros, which
    // is not a valid rotation, so set it to the identity.
    const manipInfo = newStructure(c, "experimental.tesseract_robotics.common.ManipulatorInfo");
    manipInfo.tcp_frame = "tool0";
    manipInfo.working_frame = "base_link";
    manipInfo.manipulator = "manipulator";
    manipInfo.tcp_offset = xyzRpyToTransform([0, 0, 0], [0, 0, 0]);

    // Create and configure the composite instruction
    const composite = newStructure(c, `${COMMAND_LANGUAGE}.CompositeInstruction`);
    composite.order = commandLanguageConst.CompositeInstructionOrder.ordered;
    composite.instructions = [instr1, instr2];
    composite.manipulator_info = manipInfo;
    composite.uuid = newRandomUuid();
    const compositeInput = new VarValue(composite, `${COMMAND_LANGUAGE}.CompositeInstruction`);

    // Find the first executor and get the task executor object
    const executorsInfo = (await c.getProperty("task_executors_info")) as Map<string, TaskExecutorInfo>;
    const [defaultExecutor] = executorsInfo.values();
    const executor = await c.getObjRef("task_executors", defaultExecutor!.name);

    // Create the task executor input and set the pipeline name. In this case, we are using the
    // FreespacePipeline. Other pipelines are available. These pipelines are configured when the
    // service is started.
    const execInput = newStructure(c, "experimental.tesseract_robotics.tasks.TaskExecutorInput");
    execInput.task_name = "FreespacePipeline";
    execInput.data_storage = new Map<string, VarValue>([
        ["environment", envHandleInput],
        [inputKey, compositeInput]
    ]);

    // Run the pipeline. It returns a Robot Raconteur "Generator", which is used for long running
    // operations. Read status updates from it until it is complete.
    const execGen = await executor.callGenerator<TaskExecutorStatus>("run", [execInput]);
    let status: TaskExecutorStatus | undefined;
    for await (const s of execGen) {
        status = s;
    }

    // Check the status of the pipeline to make sure it completed successfully
    if (!status || status.action_status !== actionConst.ActionStatusCode.complete) {
        throw new Error(`Planning failed with action status ${status?.action_status}`);
    }

    // Retrieve the output plan from the data storage
    const outputPlan = status.data_storage.get(outputKey)!.value as CompositeInstruction;

    // Extract and print the planned waypoints
    for (const instr of outputPlan.instructions) {
        const { waypoint } = instr.value as { waypoint: VarValue };
        const { position } = waypoint.value as { position: Float64Array };
        console.log(Array.from(position, (q) => q.toFixed(4)).join(", "));
    }

    await node.disconnectService(c);
} finally {
    await node.shutdown();
}
