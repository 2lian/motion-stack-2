"""Single-process Moonbot Zero simulator.

This is the low-level approach: you wire up every component yourself in
Python. Use this as a starting point when you need to customize the
pipeline -- plug in your own hardware driver instead of LoopBack, add
custom processing hooks, or use a different transport.

The equivalent in ROS 2 would be writing your own node from scratch
(subclassing rclpy.Node) instead of using a launch file with existing
nodes. More work, but full control.

Runs all 4 legs of the Moonbot Zero inside one process. Each joint is
simulated via LoopBack (commands are echoed back as sensor readings).
Joint data is published/subscribed as real ROS 2 topics via pyzeros (zenoh).

Run:
    pixi run mz_robot
    pixi run mz_robot --rerun

Inputs:  joint_set (pyzeros subscriber) - receives joint commands
Outputs: joint_read (pyzeros publisher) - publishes joint state
         /continuous_joint_read (pyzeros publisher) - continuous stream
"""

import asyncio
from contextlib import suppress

import asyncio_for_robotics.zenoh as afor
import pyzeros
import uvloop
from motion_stack.lvl1.core import JointCore, Lvl1Param
from motion_stack.lvl1.lvl0_loopback import LoopBack
from ms_moonbot_zero import load_moonbot_zero_urdf
from ms_pyzeros_bridge.lvl1_pyzeros import (
    Lvl1Services,
    PublisherHookJSB,
    SubscriberHookJSB,
)


@afor.scoped
async def run_leg(params: Lvl1Param, rerun=False):
    """Run a single motion stack node with all 4 legs.

    @afor.scoped creates an async scope (like a task group) that keeps
    all spawned tasks alive until the function exits or is cancelled.
    """
    scope = afor.Scope.current()

    # JointCore is the central state machine. It parses the URDF to discover
    # joints, manages their state (position, velocity, effort), and runs the
    # processing pipeline (buffering, interpolation, limits).
    core = JointCore(params)

    # Optional: start rerun visualization (web viewer + grpc).
    # This hooks into the core's internal subs to log joint data in real time.
    if rerun:
        from motion_stack.lvl1.rerun_hook import start_rerun_hook

        start_rerun_hook(scope, core, name=pyzeros.auto_session().fully_qualified_name)

    # LoopBack simulates hardware: whatever command you send is echoed back
    # as the sensor reading. Replace this with your real hardware driver
    # (e.g. a CAN bus driver, serial interface, or simulation bridge).
    lo = LoopBack(core)

    # Expose ROS services for intercompatibility with our previous version
    Lvl1Services(core)

    # Bridge the core's internal pub/sub to ROS 2 topics via pyzeros.
    # pyzeros creates real ROS 2 topics (interoperable with rclpy, rviz, etc.)
    # without needing a ROS installation -- just zenoh + rmw_zenoh.
    #
    # SubscriberHookJSB: ROS topic -> core input (receives commands)
    # PublisherHookJSB:  core output -> ROS topic (publishes state)
    SubscriberHookJSB(core.command_sub, "joint_set")
    PublisherHookJSB(core.joint_read_output, "joint_read")
    PublisherHookJSB(core.continuous_js_output, "/continuous_joint_read")

    # Start the core processing loop and the loopback simulation.
    # Both run concurrently as async tasks inside the scope.
    scope.task_group.create_task(core.run())
    scope.task_group.create_task(lo.run())

    # Keep running until cancelled (Ctrl+C).
    await asyncio.Future()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--rerun", action="store_true")
    args = parser.parse_args()

    # Compile the Moonbot Zero URDF from its jinja template.
    # The template resolves mesh paths to absolute file:// URIs.
    params = Lvl1Param(urdf=load_moonbot_zero_urdf())

    # suppress(KeyboardInterrupt) ensures Ctrl+C exits cleanly without a traceback.
    with suppress(KeyboardInterrupt):
        # pyzeros.auto_context sets up a zenoh session and registers this
        # process as a ROS 2 node "lvl1" at namespace "/".
        # Equivalent to rclpy.init() + creating a node, but without rclpy.
        with pyzeros.auto_context(node="lvl1", namespace="/"):
            # uvloop is a fast drop-in replacement for asyncio's event loop.
            uvloop.run(run_leg(params, rerun=args.rerun))
