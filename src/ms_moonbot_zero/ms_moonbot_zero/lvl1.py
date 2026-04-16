import asyncio
import os
from contextlib import suppress
from importlib.resources import as_file, files

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


@afor.scoped
async def run_leg(params: Lvl1Param, rerun=False):
    scope = afor.Scope.current()
    core = JointCore(params)

    if rerun:
        from urllib.parse import quote

        import rerun as rr
        from colorama import Fore, Style

        rr.init(pyzeros.auto_session().fully_qualified_name)
        server_uri = rr.serve_grpc()
        web_port = 9090
        rr.serve_web_viewer(
            web_port=web_port, connect_to=server_uri, open_browser=False
        )
        viewer_url = f"http://localhost:{web_port}/?url={quote(server_uri, safe='')}"
        print(
            f"\n{Style.BRIGHT}{Fore.CYAN}[rerun] open the viewer at:{Style.RESET_ALL} {viewer_url}\n",
            flush=True,
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

    await asyncio.Future()  # run until cancelled


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--rerun", action="store_true")
    args = parser.parse_args()
    params = Lvl1Param(urdf=load_moonbot_zero_urdf())
    with suppress(KeyboardInterrupt):
        with pyzeros.auto_context(node="lvl1", namespace="/"):
            uvloop.run(run_leg(params, rerun=args.rerun))
