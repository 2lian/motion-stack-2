import asyncio
from contextlib import suppress
from importlib.resources import as_file, files

import asyncio_for_robotics.zenoh as afor
import numpy as np
import pyzeros
import rerun as rr
import uvloop
from asyncio_for_robotics import BaseSub
from jinja2 import Template
from motion_stack.lvl1.core import JointCore, JointPipeline, JStateBatch, Lvl1Param
from motion_stack.lvl1.lvl0_loopback import LoopBack
from motion_stack.lvl1.rerun_bridge import RerunJointLogger
from motion_stack.lvl1.rerun_hook import Lvl1RerunHook
from motion_stack.utils.joint_state import JState, JStateBuffer
from motion_stack.utils.time import Time

from ms_pyzeros_bridge.lvl1_pyzeros import (
    Lvl1Services,
    PublisherHookJSB,
    SubscriberHookJSB,
)


@afor.scoped
async def run_leg(urdf: str):
    scope = afor.Scope.current()
    rr.init("motion_stack")
    rr.save("./rerun_viz.rrd")
    # rr.connect_grpc()
    joint_logger = RerunJointLogger(urdf)
    main_sub: BaseSub[JStateBatch] = BaseSub()
    buffer = JStateBuffer(
        JState(
            name="",
            time=Time.sn(sec=10),
            position=np.deg2rad(0.05),
            velocity=np.deg2rad(0.01),
            effort=np.deg2rad(0.001),
        ),
    )
    main_sub.asap_callback.append(lambda jsb: buffer.push(jsb))

    SubscriberHookJSB(main_sub, "/continuous_joint_read")
    async def fast():
        async for tns in afor.Rate(20).listen():
            joint_logger.sub.input_data(buffer.pull_urgent())

    async def slow():
        async for tns in afor.Rate(1/10).listen():
            joint_logger.sub.input_data(buffer.pull_new())

    scope.task_group.create_task(fast())
    scope.task_group.create_task(slow())
    scope.exit_stack.push(lambda *_: rr.get_data_recording().flush())
    await asyncio.Future()


if __name__ == "__main__":
    import argparse
    from pathlib import Path

    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=False, default=None)
    args = parser.parse_args()
    urdf = args.urdf
    if urdf and Path(urdf).is_file():
        urdf = Path(urdf).read_text()
    with suppress(KeyboardInterrupt):
        with pyzeros.auto_context(node="rerun_viz"):
            uvloop.run(run_leg(urdf))
