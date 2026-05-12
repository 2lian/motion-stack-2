"""Assembles one arm or arms one after the other in a chain"""

from typing import List, Union

import numpy as np
from scipy.spatial.transform import Rotation

from ..modules.base import RobotModule
from ..urdf.base import Joint, Link, wrap_in_robot
from .utils import link_urdf_nodes


def raw_urdf(
    modules: List[RobotModule],
    base_link_name: str = "base_link",
    xyz: np.ndarray = np.zeros(3),
) -> str:
    urdf = ""
    base_link = Link(name=base_link_name)
    urdf += base_link.compile()
    fixture = base_link.name

    for counter, module in enumerate(modules):
        fix = Joint(
            f"fix{counter}",
            fixture,
            module.urdf_assembler.root,
            xyz=xyz,
            rot=Rotation.from_rotvec(np.array([0, 0, np.pi / 2]))
            * Rotation.from_rotvec(np.array([0, np.pi, 0])),
        )
        if counter == 0:
            fix = Joint(f"fix{counter}", fixture, module.urdf_assembler.root, xyz=xyz)
        urdf += fix.compile() + module.urdf_assembler.compile()
        fixture = module.urdf_assembler.leaf

    modules[0].start_effector = base_link_name
    modules[-1].end_effector = f"{modules[-1].joint_prefix}out_straight"

    return urdf


def chain(
    arm: Union[RobotModule, List[RobotModule]],
    base_link_name: str = "base_link",
    xyz: np.ndarray = np.zeros(3),
) -> List[RobotModule]:
    """Assembles one arm or arms one after the other in a chain"""
    modules: List[RobotModule] = [arm] if not isinstance(arm, list) else arm

    urdf = wrap_in_robot(raw_urdf(modules, base_link_name=base_link_name, xyz=xyz))

    return link_urdf_nodes(modules, urdf)


if __name__ == "__main__":
    from moonbot_builder.modules.hero import MhArmV1, MhArmV2, MhArmV3

    print(raw_urdf([MhArmV1(1), MhArmV2(2), MhArmV3(3)]))
