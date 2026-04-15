import asyncio
import numpy as np
from contextlib import suppress
from importlib.resources import as_file, files

import asyncio_for_robotics.zenoh as afor
from motion_stack.utils.time import Time
import pyzeros
import rerun as rr
import uvloop
from asyncio_for_robotics import BaseSub
from jinja2 import Template
from motion_stack.lvl1.core import JointCore, JointPipeline, JStateBatch, Lvl1Param
from motion_stack.lvl1.lvl0_loopback import LoopBack
from motion_stack.lvl1.rerun_bridge import RerunJointLogger
from motion_stack.lvl1.rerun_hook import Lvl1RerunHook
from motion_stack.utils.joint_state import JState

from ms_pyzeros_bridge.lvl1_pyzeros import (
    Lvl1Services,
    PublisherHookJSB,
    SubscriberHookJSB,
)


@afor.scoped
async def run_leg(urdf: str):
    rr.init("motion_stack")
    rr.connect_grpc()
    main_sub: BaseSub[JStateBatch] = BaseSub()
    buffer = JointPipeline(
        main_sub,
        buffer_delta=JState(
            name="",
            time=Time.sn(sec=2),
            position=np.deg2rad(0.05),
            velocity=np.deg2rad(0.01),
            effort=np.deg2rad(0.001),
        ),
        batch_time=1/30
    )
    SubscriberHookJSB(main_sub, "/continuous_joint_read")
    joint_logger = RerunJointLogger(urdf)
    async for jsb in buffer.output_sub.listen_reliable():
        joint_logger.sub.input_data(jsb)


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
