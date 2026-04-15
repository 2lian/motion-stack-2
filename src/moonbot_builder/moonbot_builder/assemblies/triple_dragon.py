from typing import List

import numpy as np
from launch_ros.actions import Node
from scipy.spatial.transform import Rotation

from ..modules.base import RobotModule
from ..urdf.base import Joint, Link, wrap_in_robot
from .utils import link_urdf_nodes


def raw_urdf(
    manip_wheel: RobotModule,
    manipulator: RobotModule,
    front_wheel: RobotModule,
    bridge: RobotModule,
    back_wheel: RobotModule,
) -> str:
    urdf = ""
    base_link = Link(name="base_link")
    urdf += base_link.compile()
    fixture = base_link.name

    base_rot = Rotation.from_rotvec(
        np.array([0, np.deg2rad(-45), 0]),
        degrees=False,
    )

    fix1 = Joint(
        "fix1",
        fixture,
        manipulator.urdf_assembler.root,
        rot=Rotation.from_rotvec(np.array([0, 0, np.pi])) * base_rot ,
    )
    urdf += fix1.compile() + manipulator.urdf_assembler.compile()

    fix2 = Joint(
        "fix2",
        fixture,
        front_wheel.urdf_assembler.root,
        rot=base_rot,
    )
    urdf += fix2.compile() + front_wheel.urdf_assembler.compile()

    fix3 = Joint("fix3", front_wheel.urdf_assembler.leaf, bridge.urdf_assembler.root)
    urdf += fix3.compile() + bridge.urdf_assembler.compile()

    fix4 = Joint("fix4", bridge.urdf_assembler.leaf, back_wheel.urdf_assembler.root)
    urdf += fix4.compile() + back_wheel.urdf_assembler.compile()

    bridge.end_effector = back_wheel.urdf_assembler.leaf

    fix0 = Joint(
        "fix0",
        manipulator.urdf_assembler.leaf,
        manip_wheel.urdf_assembler.root,
        rot=Rotation.from_rotvec(np.array([0, 0, 0])),
    )
    urdf += fix0.compile() + manip_wheel.urdf_assembler.compile()

    # manipulator.end_effector = manip_wheel.urdf_assembler.leaf

    return urdf


def triple_dragon(
    manip_wheel: RobotModule,
    manipulator: RobotModule,
    front_wheel: RobotModule,
    bridge: RobotModule,
    back_wheel: RobotModule,
) -> List[Node]:
    modules = [manip_wheel, manipulator, front_wheel, bridge, back_wheel]

    urdf = wrap_in_robot(raw_urdf(*modules))

    return link_urdf_nodes(modules, urdf)
