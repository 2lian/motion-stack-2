import asyncio
from contextlib import suppress
from importlib.resources import as_file, files
from pprint import pprint

import asyncio_for_robotics.zenoh as afor
import colorama
import rerun as rr
from jinja2 import Template
from motion_stack.lvl1.core import JointCore, JState, Lvl1Param, Time
from motion_stack.lvl1.joint_api import AsyncJointSyncer
from motion_stack.lvl1.lvl0_loopback import LoopBack
from motion_stack.lvl1.rerun_hook import Lvl1RerunHook
from ms_pyzeros_bridge.lvl1_pyzeros import (
    Lvl1Services,
    PublisherHookJSB,
    SubscriberHookJSB,
)
from pyzeros.node import Node


async def main():
    async with asyncio.TaskGroup() as tg:
        syncer = AsyncJointSyncer()

        # creates ros node
        node = Node(
            name=f"operator",
            namespace=f"",
        )
        # bind it to this TaskGroup for cleanup
        tg.create_task(node.async_bind())

        # subscribe to ros, and pushes incomming data to the syncer
        for limb_n in range(1, 5):
            namespace = f"leg{limb_n}"
            joint_set_pub = PublisherHookJSB(
                syncer.command_output, node, f"/{namespace}/joint_set"
            )
            joint_read_sub = SubscriberHookJSB(
                syncer.sensor_input, node, f"/{namespace}/joint_read"
            )
            # "spins" the sub onto this taskgroup
            tg.create_task(joint_set_pub.run())
            tg.create_task(joint_read_sub.run())
        # "spins" the syncer onto this taskgroup
        tg.create_task(syncer.run())

        # and we can use the api as usual
        joints = {f"joint{a}_{b}" for a in range(1, 5) for b in range(1, 4)}
        print(f"{colorama.Fore.BLUE}Waiting {colorama.Fore.RESET} for {joints=}")
        await syncer.wait_ready(joints)

        print(f"{colorama.Fore.RED}SYNCER READY :){colorama.Fore.RESET}")
        while 1:
            await syncer.lerp(
                {f"joint{a}_{b}": 0.5 for a in range(1, 5) for b in range(1, 4)}
            )
            print(f"{colorama.Fore.RED}OH :){colorama.Fore.RESET}")
            await syncer.lerp(
                {f"joint{a}_{b}": -0.2 for a in range(1, 5) for b in range(1, 4)}
            )
            print(f"{colorama.Fore.RED}WOW :){colorama.Fore.RESET}")


if __name__ == "__main__":
    ses = afor.auto_session()
    try:
        with suppress(KeyboardInterrupt):
            asyncio.run(main())
    finally:
        ses.close()
