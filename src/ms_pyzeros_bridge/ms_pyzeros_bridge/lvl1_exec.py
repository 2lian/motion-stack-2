import asyncio
from contextlib import suppress

import asyncio_for_robotics.zenoh as afor
import pyzeros
import uvloop
from motion_stack.lvl1.core import JointCore, Lvl1Param
from motion_stack.lvl1.lvl0_loopback import LoopBack

from ms_pyzeros_bridge.lvl1_pyzeros import (
    Lvl1Services,
    PublisherHookJSB,
    SubscriberHookJSB,
)


@afor.scoped
async def run_leg(params: Lvl1Param, rerun=False):
    scope = afor.Scope.current()
    core = JointCore(params)

    if rerun:
        import os

        from motion_stack.lvl1.rerun_hook import start_rerun_hook

        bag_name = pyzeros.auto_session().fully_qualified_name
        os.makedirs("./bag", exist_ok=True)
        start_rerun_hook(
            scope,
            core,
            name=bag_name,
            save_path=f"./bag/{bag_name.replace('/', '_')}.rrd",
        )

    lo = LoopBack(core)

    Lvl1Services(core)
    SubscriberHookJSB(core.command_sub, "joint_set")
    PublisherHookJSB(core.joint_read_output, "joint_read")
    PublisherHookJSB(core.continuous_js_output, "/continuous_joint_read")

    scope.task_group.create_task(core.run())
    scope.task_group.create_task(lo.run())

    params.urdf = ""

    await asyncio.Future()  # run until cancelled


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--ms-lvl1-json", required=True)
    parser.add_argument("--rerun", action="store_true")
    parser.add_argument("--ms-additional-json", required=False, default=None)
    args = parser.parse_args()
    params = Lvl1Param.from_json(args.ms_lvl1_json)
    with suppress(KeyboardInterrupt):
        with pyzeros.auto_context(node="lvl1", namespace=params.namespace):
            uvloop.run(run_leg(params, rerun=args.rerun))
