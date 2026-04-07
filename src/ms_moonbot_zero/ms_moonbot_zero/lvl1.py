import asyncio
from importlib.resources import as_file, files

import rerun as rr
from asyncio_for_robotics import BaseSub
from jinja2 import Template
from motion_stack.lvl0_loopback import LoopBack
from motion_stack.lvl1 import JointCore, Lvl1Param
from motion_stack.lvl1_rerun_hook import Lvl1RerunHook
from ms_pyzeros_bridge.lvl1_pyzeros import Bridge


def load_moonbot_zero_urdf() -> str:
    assets = files("ms_moonbot_zero").joinpath("assets")
    template = Template(assets.joinpath("moonbot_zero.jinja.urdf").read_text())

    with as_file(assets) as assets_dir:
        mesh_path = assets_dir / "meshes"
        return template.render(
            root_path=assets_dir.as_uri(),
            mesh_path=mesh_path.as_uri(),
        )


params = Lvl1Param(urdf=load_moonbot_zero_urdf())


async def main():
    rr.init("moonbot_zero")
    rr.connect_grpc()
    command_sub = BaseSub()
    sensor_sub = BaseSub()
    core = JointCore(sensor_sub, command_sub, params)
    lo = LoopBack(core)
    rr_hook = Lvl1RerunHook(core)
    bridge = Bridge(core)

    async with asyncio.TaskGroup() as tg:
        tg.create_task(core.run())
        tg.create_task(lo.run())
        tg.create_task(rr_hook.run())
        tg.create_task(bridge.run())


if __name__ == "__main__":
    asyncio.run(main())
