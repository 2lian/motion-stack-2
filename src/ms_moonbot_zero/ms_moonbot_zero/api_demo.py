"""API client demo: connects to a running Moonbot Zero and moves all joints.

This is what user/application code typically looks like. You don't set up
any core or hardware -- you just connect to the topics published by a
running robot and send commands through the AsyncJointSyncer API.

This is like writing a ROS 2 client node that subscribes to /joint_states
and publishes to /joint_commands, without caring how the robot node is
implemented. pyzeros creates real ROS 2 topics, so this client is fully
interoperable with rclpy nodes, rviz, rosbag, etc.

Requires robot_demo.py (or mz_launcher.py) to be running first.

Run:
    pixi run mz_api

Inputs:  joint_read (pyzeros subscriber) - reads current joint state
Outputs: joint_set (pyzeros publisher) - sends joint commands
"""

import asyncio
from contextlib import suppress

import asyncio_for_robotics.zenoh as afor
import colorama
import pyzeros
from motion_stack.lvl1.joint_api import AsyncJointSyncer
from ms_pyzeros_bridge.lvl1_pyzeros import PublisherHookJSB, SubscriberHookJSB


@afor.scoped
async def main():
    scope = afor.Scope.current()

    # AsyncJointSyncer is the high-level API for controlling joints.
    # It buffers sensor readings, tracks which joints exist, and provides
    # methods like lerp() to move joints smoothly to a target position.
    # Think of it as a MoveIt-lite for joint-space control.
    syncer = AsyncJointSyncer()

    # Connect to joint topics on multiple namespaces.
    # We subscribe to both "" (root, for single-process robot_demo.py)
    # and "leg1"-"leg4" (for multi-process mz_launcher.py).
    # This way the same client works with either mode.
    for s, p in [
        (f"{namespace}/joint_read", f"{namespace}/joint_set")
        for namespace in ["", "leg1", "leg2", "leg3", "leg4"]
    ]:
        # SubscriberHookJSB bridges a ROS 2 topic into the syncer's input.
        # It tracks which joint names have been seen on this topic.
        _sh = SubscriberHookJSB(syncer.sensor_input, s)

        # PublisherHookJSB bridges the syncer's output to a ROS 2 topic.
        _ph = PublisherHookJSB(syncer.command_output, p)

        # filter = _sh.seen ensures we only publish commands to joints that
        # actually exist on this namespace. Without this, commands for leg1
        # joints would also be sent to the leg2 topic (and ignored).
        _ph.filter = _sh.seen
        print(
            "Interfacing with: ",
            _sh.pyz_sub.fully_qualified_name,
            _ph.pyz_pub.fully_qualified_name,
        )

    # Start the syncer's internal processing loop.
    scope.task_group.create_task(syncer.run())

    # Define the set of joints we expect (4 legs x 3 joints each = 12).
    joints = {f"joint{a}_{b}" for a in range(1, 5) for b in range(1, 4)}

    # Block until all 12 joints have reported at least one reading.
    # This ensures the robot is fully online before we start sending commands.
    print(f"{colorama.Fore.BLUE}Waiting {colorama.Fore.RESET} for {joints=}")
    await syncer.wait_ready(joints)

    # Move all joints back and forth between two positions.
    # lerp() smoothly interpolates from current position to target,
    # and awaits until all joints reach the target (within tolerance).
    print(f"{colorama.Fore.RED}SYNCER READY :){colorama.Fore.RESET}")
    while 1:
        print("going to -0.2")
        await syncer.lerp({j: -0.2 for j in joints})
        print("going to 1.0")
        await syncer.lerp({j: 1 for j in joints})


if __name__ == "__main__":
    # suppress(KeyboardInterrupt) ensures Ctrl+C exits cleanly.
    with suppress(KeyboardInterrupt):
        # Register as ROS 2 node "operator" -- no namespace needed for a client.
        with pyzeros.auto_context(node="operator"):
            asyncio.run(main())
