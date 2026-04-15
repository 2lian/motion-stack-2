import argparse
from pprint import pprint
import sys
from subprocess import Popen, TimeoutExpired
from tempfile import NamedTemporaryFile

import ujson

from moonbot_builder.assemblies.parallel import mega_palet
from moonbot_builder.modules.base import RobotModule
from moonbot_builder.modules.hero import (
    MhArmV1,
    MhArmV2,
    MhArmV3,
    MhWheelV1,
    MhWheelV2,
    MhWheelV3,
)

from .assemblies.dragon import dragon


def build_commands(modules: list[RobotModule]) -> list[list[str]]:
    cmds = []
    for m in modules:
        for node in m.get_lvl1():
            cmds.append(
                m.executable
                + [
                    "--ms-lvl1-json",
                    node.ms_params.to_json(),
                    "--ms-additional-json",
                    ujson.dumps(node.other_params),
                ]
            )
    return cmds


def run_all(cmds: list[list[str]]) -> None:
    procs = [Popen(cmd, text=True) for cmd in cmds]
    try:
        for p in procs:
            p.wait()
    except KeyboardInterrupt:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=5)
            except TimeoutExpired:
                p.kill()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--viz", action="store_true")
    args = parser.parse_args()

    modules = mega_palet(9)

    if args.viz:
        run_all(
            [[sys.executable, "-m", "ms_pyzeros_bridge.rerun_viz", "--urdf", modules[0].compile_urdf()]]
        )
    else:
        run_all(build_commands(modules))
