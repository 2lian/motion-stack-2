"""Launches the 4 legs of the Moonbot Zero as separate lvl1_exec processes.

Follows the same Popen pattern as moonbot_builder.launch.

Usage:
    pixi run python ./src/ms_moonbot_zero/ms_moonbot_zero/mz_launcher.py
    pixi run python ./src/ms_moonbot_zero/ms_moonbot_zero/mz_launcher.py --rerun
    pixi run python ./src/ms_moonbot_zero/ms_moonbot_zero/mz_launcher.py --viz
"""

import argparse
import sys
from subprocess import Popen, TimeoutExpired
from tempfile import NamedTemporaryFile

from importlib.resources import as_file, files
from jinja2 import Template
from motion_stack.lvl1.core import Lvl1Param


LEGS = [
    {"namespace": "leg1", "end_effector_name": "end1"},
    {"namespace": "leg2", "end_effector_name": "end2"},
    {"namespace": "leg3", "end_effector_name": "end3"},
    {"namespace": "leg4", "end_effector_name": "end4"},
]


def load_moonbot_zero_urdf() -> str:
    """Compiles the jinja with absolute file:// paths."""
    assets = files("ms_moonbot_zero").joinpath("assets")
    template = Template(assets.joinpath("moonbot_zero.jinja.urdf").read_text())
    with as_file(assets) as assets_dir:
        mesh_path = assets_dir / "meshes"
        return template.render(
            root_path=assets_dir.as_uri(),
            mesh_path=mesh_path.as_uri(),
        )


def build_commands(urdf_path: str) -> list[list[str]]:
    cmds = []
    for leg in LEGS:
        params = Lvl1Param(
            urdf=urdf_path,
            namespace=leg["namespace"],
            end_effector_name=leg["end_effector_name"],
            ignore_limits=True,
        )
        cmd = [
            sys.executable,
            "-m",
            "ms_pyzeros_bridge.lvl1_exec",
            "--ms-lvl1-json",
            params.to_json(),
        ]
        cmds.append(cmd)
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
    parser = argparse.ArgumentParser(description="Launch Moonbot Zero (4 legs)")
    parser.add_argument("--viz", action="store_true", help="Launch rerun visualizer only")
    args = parser.parse_args()

    urdf = load_moonbot_zero_urdf()

    # Write URDF to temp file so lvl1_exec can read it by path
    f = NamedTemporaryFile(mode="w", suffix=".urdf", prefix="mz_", delete=False)
    f.write(urdf)
    f.close()

    if args.viz:
        run_all(
            [[sys.executable, "-m", "ms_pyzeros_bridge.rerun_viz", "--urdf", urdf]]
        )
    else:
        cmds = build_commands(f.name)
        print(f"Launching {len(cmds)} leg process(es)...")
        for leg, cmd in zip(LEGS, cmds):
            print(f"  Namespace: {leg['namespace']} | End_effector: {leg['end_effector_name']}")
        run_all(cmds)
