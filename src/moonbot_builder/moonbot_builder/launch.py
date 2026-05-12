import argparse
import sys
from pprint import pprint
from subprocess import Popen, TimeoutExpired
from tempfile import NamedTemporaryFile

import ujson

import moonbot_builder.urdf.hero as hero_urdf
from moonbot_builder.assemblies.chain import chain
from moonbot_builder.assemblies.dragon import dragon
from moonbot_builder.assemblies.minimal import minimal
from moonbot_builder.assemblies.parallel import mega_palet
from moonbot_builder.assemblies.tricycle import tricycle
from moonbot_builder.assemblies.triple_dragon import triple_dragon
from moonbot_builder.assemblies.vehicle import vehicle
from moonbot_builder.modules.base import RobotModule
from moonbot_builder.modules.hero import (
    MhArmV1,
    MhArmV2,
    MhArmV3,
    MhBody,
    MhWheelV1,
    MhWheelV2,
    MhWheelV3,
)
from moonbot_builder.modules.gusta import MgArmV3
from moonbot_builder.urdf.hero import MESH_TYPE


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

    hero_urdf.DEFAULT_MESH_TYPE = MESH_TYPE.DAE
    # modules = mega_palet(5)

    # modules = minimal(MhArmV1(1), MhWheelV1(1))
    modules = vehicle(MhWheelV1(1), MhArmV2(1), MhWheelV2(1))
    # modules = chain(MgArmV3(1))
    # modules = chain([MgArmV3(1), MgArmV3(2)])
    # modules = chain(
    #     [m(k) for k in range(3) for m in [MgArmV3, MhArmV1, MhArmV2, MhArmV3] ]
    # )
    # modules = dragon(MhArmV1(1), MhWheelV1(1), MhArmV3(1), MhWheelV3(1))
    # modules = tricycle(
    #     MhWheelV1(1),
    #     MhWheelV1(2),
    #     MhWheelV1(3),
    #     MhArmV2(1),
    #     MhArmV2(2),
    #     MhArmV2(3),
    #     MhBody(1),
    #     manipulator=MhArmV1(1),
    # )

    if args.viz:
        run_all(
            [
                [
                    sys.executable,
                    "-m",
                    "ms_pyzeros_bridge.rerun_viz",
                    "--urdf",
                    modules[0].compile_urdf(),
                ]
            ]
        )
    else:
        run_all(build_commands(modules))
