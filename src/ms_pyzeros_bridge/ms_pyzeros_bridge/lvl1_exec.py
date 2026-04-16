import asyncio
from contextlib import suppress
from importlib.resources import as_file, files
import os

import asyncio_for_robotics.zenoh as afor
import pyzeros
import uvloop
from jinja2 import Template
from motion_stack.lvl1.core import JointCore, Lvl1Param
from motion_stack.lvl1.lvl0_loopback import LoopBack
from motion_stack.lvl1.rerun_hook import Lvl1RerunHook

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
        import rerun as rr

        rr.init(pyzeros.auto_session().fully_qualified_name)
        os.makedirs("./bag", exist_ok=True)
        rr.save(
            f"./bag/{pyzeros.auto_session().fully_qualified_name.replace("/","_")}.rrd"
        )
        rr_hook = Lvl1RerunHook(core)
        scope.task_group.create_task(rr_hook.run())

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
