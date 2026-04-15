import asyncio
import uvloop
from contextlib import suppress
from importlib.resources import as_file, files

import asyncio_for_robotics.zenoh as afor
import pyzeros
import rerun as rr
from jinja2 import Template
from motion_stack.lvl1.core import JointCore, Lvl1Param
from motion_stack.lvl1.lvl0_loopback import LoopBack
from motion_stack.lvl1.rerun_hook import Lvl1RerunHook
from ms_pyzeros_bridge.lvl1_pyzeros import (
    Lvl1Services,
    PublisherHookJSB,
    SubscriberHookJSB,
)


def load_moonbot_zero_urdf() -> str:
    """Compiles the jinja with absolute path (file:// instead of ros package://)."""
    assets = files("ms_moonbot_zero").joinpath("assets")
    template = Template(assets.joinpath("moonbot_zero.jinja.urdf").read_text())
    with as_file(assets) as assets_dir:
        mesh_path = assets_dir / "meshes"
        return template.render(
            root_path=assets_dir.as_uri(),
            mesh_path=mesh_path.as_uri(),
        )


async def run_leg(limb_n: int):
    with pyzeros.auto_context(node="lvl1", namespace=f"leg{limb_n+1}"):
        async with afor.Scope() as scope:
            params = Lvl1Param(
                urdf=load_moonbot_zero_urdf(),
                end_effector_name=limb_n,
            )
            core = JointCore(params)
            lo = LoopBack(core)
            rr_hook = Lvl1RerunHook(core)

            Lvl1Services(core)
            SubscriberHookJSB(core.command_sub, "joint_set")
            PublisherHookJSB(core.joint_read_output, "joint_read")

            scope.task_group.create_task(core.run())
            scope.task_group.create_task(lo.run())
            scope.task_group.create_task(rr_hook.run())

            await asyncio.Future()  # run until cancelled


async def main():
    rr.init("moonbot_zero")
    rr.connect_grpc()

    async with asyncio.TaskGroup() as tg:
        for limb_n in range(4):
            tg.create_task(run_leg(limb_n))


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        uvloop.run(main())
