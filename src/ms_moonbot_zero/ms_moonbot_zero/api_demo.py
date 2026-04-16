import asyncio
from contextlib import suppress
from typing import Tuple

import asyncio_for_robotics.zenoh as afor
import colorama
import pyzeros
import zenoh
from motion_stack.lvl1.joint_api import AsyncJointSyncer, make_delta_time
from ms_pyzeros_bridge.lvl1_pyzeros import PublisherHookJSB, SubscriberHookJSB


@afor.scoped
async def main():
    scope = afor.Scope.current()
    syncer = AsyncJointSyncer()

    _sh = SubscriberHookJSB(syncer.sensor_input, "joint_read")
    _ph = PublisherHookJSB(syncer.command_output, "joint_set")
    _ph.filter = _sh.seen
    scope.task_group.create_task(syncer.run())

    joints = {f"joint{a}_{b}" for a in range(1, 5) for b in range(1, 4)}

    print(f"{colorama.Fore.BLUE}Waiting {colorama.Fore.RESET} for {joints=}")
    await syncer.wait_ready(joints)

    print(f"{colorama.Fore.RED}SYNCER READY :){colorama.Fore.RESET}")
    while 1:
        print("going to -0.2")
        await syncer.lerp({j: -0.2 for j in joints})
        print("going to 1.0")
        await syncer.lerp({j: 1 for j in joints})


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        with pyzeros.auto_context(node="operator"):
            asyncio.run(main())
