"""Multi-process launcher for the Moonbot Zero.

This is the equivalent of a ROS 2 launch file, but in plain Python.
Instead of XML/YAML launch descriptions, we build the parameters in Python
and spawn each node as a subprocess with Popen.

Each leg runs as its own process (a ROS 2 node created via pyzeros),
publishing real ROS 2 topics over zenoh. The URDF and parameters are
serialized to JSON and passed on the command line -- similar to how
ros2 launch passes parameters to nodes.

Run:
    pixi run mz_launcher
    pixi run mz_launcher --viz

Outputs: 4 subprocesses, each publishing on their own namespace
         (leg1/joint_read, leg2/joint_read, etc.)
"""

import argparse
import sys
from subprocess import Popen, TimeoutExpired

from motion_stack.lvl1.core import Lvl1Param
from ms_moonbot_zero import load_moonbot_zero_urdf


# Each leg has its own ROS namespace and end effector.
# The namespace isolates topics: leg1/joint_read, leg2/joint_read, etc.
LEGS = [
    {"namespace": "leg1", "end_effector_name": "end1"},
    {"namespace": "leg2", "end_effector_name": "end2"},
    {"namespace": "leg3", "end_effector_name": "end3"},
    {"namespace": "leg4", "end_effector_name": "end4"},
]


def build_commands(params_list: list[Lvl1Param]) -> list[list[str]]:
    """Convert a list of Lvl1Param into CLI commands for lvl1_exec.

    Each command is equivalent to running:
        python -m ms_pyzeros_bridge.lvl1_exec --ms-lvl1-json '{"urdf": "...", ...}'

    This is similar to how ros2 launch builds node commands with parameters,
    except parameters are passed as a JSON string instead of a YAML file.

    sys.executable ensures we use the same Python interpreter (important
    when running inside pixi or a virtualenv).
    """
    cmds = []
    for params in params_list:
        cmd = [
            # sys.executable = path to the current Python interpreter
            # (e.g. .pixi/envs/default/bin/python)
            sys.executable,
            # -m runs a module as a script, like: python -m http.server
            "-m",
            "ms_pyzeros_bridge.lvl1_exec",
            # The node reads all its config (URDF, namespace, etc.) from this JSON
            "--ms-lvl1-json",
            params.to_json(),
        ]
        cmds.append(cmd)
    return cmds


def run_all(cmds: list[list[str]]) -> None:
    """Spawn all commands as subprocesses, terminate on Ctrl+C.

    Popen starts each process without blocking -- all legs run in parallel.
    We then wait for any process to exit. On Ctrl+C, we cleanly terminate
    all processes (SIGTERM), then force-kill after 5 seconds if needed.
    """
    procs = [Popen(cmd, text=True) for cmd in cmds]
    try:
        for p in procs:
            p.wait()
    except KeyboardInterrupt:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=5)
            except TimeoutExpired:
                p.kill()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Launch Moonbot Zero (4 legs)")
    parser.add_argument("--viz", action="store_true", help="Launch rerun visualizer only")
    args = parser.parse_args()

    # Compile the Moonbot Zero URDF from its jinja template.
    # This resolves mesh paths to absolute file:// URIs so each subprocess
    # can find the STL files regardless of its working directory.
    urdf = load_moonbot_zero_urdf()

    # Build one Lvl1Param per leg. All legs share the same URDF, but each
    # gets a different namespace and end effector so the motion stack knows
    # which kinematic chain to manage.
    # This is like setting <param> in a ROS launch file.
    params_list = [
        Lvl1Param(
            urdf=urdf,
            namespace=leg["namespace"],
            end_effector_name=leg["end_effector_name"],
            ignore_limits=True,
        )
        for leg in LEGS
    ]

    if args.viz:
        # Launch the rerun visualizer only (no robot processes).
        # Pass the URDF so rerun can render the 3D model.
        run_all(
            [[sys.executable, "-m", "ms_pyzeros_bridge.rerun_viz", "--urdf", urdf]]
        )
    else:
        # Build the CLI commands and spawn one process per leg.
        cmds = build_commands(params_list)
        print(f"Launching {len(cmds)} leg process(es)...")
        for p in params_list:
            print(f"  Namespace: {p.namespace} | End_effector: {p.end_effector_name}")
        run_all(cmds)
