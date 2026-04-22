import asyncio
from contextlib import suppress

import asyncio_for_robotics.zenoh as afor
import uvloop
from motion_stack.lvl1.core import JointCore, Lvl1Param
from motion_stack.lvl1.lvl0_loopback import LoopBack

from ms_zenoh_bridge.lvl1_zenoh import PublisherHookJSB, SubscriberHookJSB


@afor.scoped
async def run_leg(params: Lvl1Param, rerun: bool = False):
    scope = afor.Scope.current()
    core = JointCore(params)

    if rerun:
        import os

        from motion_stack.lvl1.rerun_hook import start_rerun_hook

        os.makedirs("./bag", exist_ok=True)
        start_rerun_hook(
            scope,
            core,
            name=params.namespace,
            save_path=f"./bag/{params.namespace.replace('/', '_')}.rrd",
        )

    lo = LoopBack(core)

    namespace = params.namespace.strip("/")

    SubscriberHookJSB(core.command_sub, f"{namespace}/joint_set")
    PublisherHookJSB(core.joint_read_output, f"{namespace}/joint_read")
    PublisherHookJSB(core.continuous_js_output, f"continuous_joint_read")

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
        uvloop.run(run_leg(params, rerun=args.rerun))
