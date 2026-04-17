from typing import List

import numpy as np
from scipy.spatial.transform import Rotation

from ..modules.base import RobotModule
from ..urdf.base import Joint, Link, wrap_in_robot
from .utils import link_urdf_nodes


def raw_urdf(
    arm: RobotModule,
    wheel: RobotModule,
    base_link_name: str = "base_link",
) -> str:
    urdf = ""
    base_link = Link(name=base_link_name)
    urdf += base_link.compile()
    fixture = base_link.name

    end_effector_link = Link(name=f"{arm.namespace}_gripper2_straight")
    ee_rot = Rotation.from_rotvec(
        np.array([0, np.deg2rad(90), 0]), degrees=False
    ) * Rotation.from_rotvec(np.array([0, 0, np.deg2rad(180)]), degrees=False)

    fix1 = Joint("fix1", fixture, arm.urdf_assembler.root)
    urdf += fix1.compile() + arm.urdf_assembler.compile()

    ee_straight = Joint(
        "fix_ee",
        arm.urdf_assembler.leaf,
        end_effector_link.name,
        rot=ee_rot,
    )
    urdf += end_effector_link.compile() + ee_straight.compile()

    fix2 = Joint("fix2", fixture, wheel.urdf_assembler.root)
    urdf += fix2.compile() + wheel.urdf_assembler.compile()

    arm.end_effector = end_effector_link.name
    arm.start_effector = base_link.name

    return urdf


def minimal(
    arm: RobotModule,
    wheel: RobotModule,
    base_link_name: str = "base_link",
) -> List[RobotModule]:
    modules = [arm, wheel]

    urdf = wrap_in_robot(raw_urdf(*modules, base_link_name=base_link_name))

    return link_urdf_nodes(modules, urdf)
