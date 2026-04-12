import asyncio
from contextlib import suppress

import asyncio_for_robotics.zenoh as afor
import colorama
import pyzeros
from motion_stack.lvl1.joint_api import AsyncJointSyncer
from ms_pyzeros_bridge.lvl1_pyzeros import (
    PublisherHookJSB,
    SubscriberHookJSB,
)


async def main():
    with pyzeros.auto_context(node="operator"):
        async with afor.Scope() as scope:
            syncer = AsyncJointSyncer()

            for limb_n in range(1, 5):
                namespace = f"leg{limb_n}"
                joint_set_pub = PublisherHookJSB(
                    syncer.command_output, f"/{namespace}/joint_set"
                )
                joint_read_sub = SubscriberHookJSB(
                    syncer.sensor_input, f"/{namespace}/joint_read"
                )
                scope.task_group.create_task(joint_set_pub.run())
                scope.task_group.create_task(joint_read_sub.run())
            scope.task_group.create_task(syncer.run())

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
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
