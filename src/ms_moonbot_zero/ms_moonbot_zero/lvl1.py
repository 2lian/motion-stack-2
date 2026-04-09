import asyncio
from contextlib import suppress
from importlib.resources import as_file, files

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


def load_moonbot_zero_urdf() -> str:
    """Compiles the jinja with absolute path. this means file:// instead of
    package:// that is ros specific.
    """
    assets = files("ms_moonbot_zero").joinpath("assets")
    template = Template(assets.joinpath("moonbot_zero.jinja.urdf").read_text())

    with as_file(assets) as assets_dir:
        mesh_path = assets_dir / "meshes"
        return template.render(
            root_path=assets_dir.as_uri(),
            mesh_path=mesh_path.as_uri(),
        )


async def main():
    ##### ===  SETUP === #####

    # sets up rerun. You have to launch `pixi run rerun` or the grpc will block
    rr.init("moonbot_zero")
    rr.connect_grpc()
    # Task group for lifetime handling
    async with asyncio.TaskGroup() as tg:
        cores: list[JointCore] = []
        # each limb can be in different processes, but doesn't need to
        # easier to put them all here for this example, Unga Bunga
        for limb_n in range(4):
            # parameters for the limb
            params = Lvl1Param(
                urdf=load_moonbot_zero_urdf(),
                end_effector_name=limb_n,  # only end-effector number changes, just like motion stack 1. Those params can come from a json file
            )

            ##### ===  Internals === #####

            # motion stack core with many hooks and subscribers
            core = JointCore(params)
            cores.append(core)
            # Hooks onto lvl0 I/O simulating a 60Hz motor->sensor
            lo = LoopBack(core)
            # Hooks onto core to log data onto rerun
            rr_hook = Lvl1RerunHook(core)

            ##### ===  ROS 2 (through pyzeros) === #####

            # creates the node
            node = Node(
                name=f"lvl1_{limb_n+1}",
                namespace=f"leg{limb_n+1}",  # has to be named legX for
                # compatibility with TUI
            )
            # creates the alive services for compatibility with motion stack 1 API
            lvl1_srvs = Lvl1Services(core, node)
            # creates a ROS 2 subscriber that will input data into
            # core.command_sub internal subscriber
            joint_set_sub = SubscriberHookJSB(core.command_sub, node, "joint_set")
            # creates a ROS 2 publisher that will publish every data emitted
            # onto core.joint_read_output
            joint_read_pub = PublisherHookJSB(
                core.joint_read_output, node, "joint_read"
            )

            ##### === Lifetimes === #####

            # First async functions! up until now (and still now) everything is
            # synchronous.
            # Those will spin/consume/execute the participants/pub/sub/srvs/tasks
            # onto this taskgroup, so if one crashes, they all stop and bubble
            # up the error.
            # they also handle the lifetime of participants (pub/sub/srv/node)
            # When the task finishes/crashes, everything is closed/undeclared.
            # Therefore it is impossible to forget to close the node/pub/sub,
            # exiting this TaskGroup block ensures everything is dead.
            tg.create_task(core.run())
            tg.create_task(lo.run())
            tg.create_task(rr_hook.run())
            tg.create_task(lvl1_srvs.run())
            tg.create_task(joint_set_sub.run())
            tg.create_task(joint_read_pub.run())
        # await sync_traj(cores)


if __name__ == "__main__":
    ses = afor.auto_session()
    try:
        with suppress(KeyboardInterrupt):
            asyncio.run(main())
    finally:
        ses.close()
