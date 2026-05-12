from typing import List

import numpy as np
from scipy.spatial.transform import Rotation

from ..modules.base import RobotModule
from ..urdf.base import Joint, Link, wrap_in_robot
from .utils import link_urdf_nodes


def raw_urdf(
    front_wheel: RobotModule,
    bridge: RobotModule,
    back_wheel: RobotModule,
    base_link_name: str = "base_link",
) -> str:
    urdf = ""
    base_link = Link(name=base_link_name)
    urdf += base_link.compile()
    fixture = base_link.name

    base_rot = Rotation.from_rotvec(
        np.array([0, np.deg2rad(-45), 0]),
        degrees=False,
    ) * Rotation.from_rotvec(
        np.array([0, 0, np.deg2rad(180)]),
        degrees=False,
    )
    base_xyz = np.array([0.113, 0.0, 0.113])

    fix2 = Joint(
        "fix2",
        fixture,
        front_wheel.urdf_assembler.root,
        xyz=base_xyz,
        rot=base_rot,
    )
    urdf += fix2.compile() + front_wheel.urdf_assembler.compile()

    fix3 = Joint("fix3", front_wheel.urdf_assembler.leaf, bridge.urdf_assembler.root)
    urdf += fix3.compile() + bridge.urdf_assembler.compile()

    fix4 = Joint("fix4", bridge.urdf_assembler.leaf, back_wheel.urdf_assembler.root)
    urdf += fix4.compile() + back_wheel.urdf_assembler.compile()

    c45 = Link(f"{back_wheel.urdf_assembler.prefix}center45")
    toc45 = Joint(
        "toc45",
        back_wheel.urdf_assembler.root,
        c45.name,
        xyz=np.array([0, 0, -0.155]),
        rot=base_rot,
    )
    urdf += c45.compile() + toc45.compile()

    bridge.end_effector = c45.name
    bridge.start_effector = base_link.name
    return urdf


def vehicle(
    front_wheel: RobotModule,
    bridge: RobotModule,
    back_wheel: RobotModule,
    base_link_name: str = "base_link",
) -> List[RobotModule]:
    modules = [front_wheel, bridge, back_wheel]

    urdf = wrap_in_robot(raw_urdf(*modules, base_link_name=base_link_name))

    return link_urdf_nodes(modules, urdf)


if __name__ == "__main__":
    from moonbot_builder.modules.hero import MhArmV1, MhWheelV1, MhWheelV3

    print(raw_urdf(MhWheelV1(1), MhArmV1(1), MhWheelV3(1)))
