from typing import List

from launch_ros.actions import Node

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

    fix2 = Joint("fix2", fixture, front_wheel.urdf_assembler.root)
    urdf += fix2.compile() + front_wheel.urdf_assembler.compile()

    fix3 = Joint("fix3", front_wheel.urdf_assembler.leaf, bridge.urdf_assembler.root)
    urdf += fix3.compile() + bridge.urdf_assembler.compile()

    fix4 = Joint("fix4", bridge.urdf_assembler.leaf, back_wheel.urdf_assembler.root)
    urdf += fix4.compile() + back_wheel.urdf_assembler.compile()

    bridge.end_effector = back_wheel.urdf_assembler.leaf
    return urdf


def vehicule(
    front_wheel: RobotModule,
    bridge: RobotModule,
    back_wheel: RobotModule,
    base_link_name: str = "base_link",
) -> List[Node]:
    modules = [front_wheel, bridge, back_wheel]

    urdf = wrap_in_robot(raw_urdf(*modules, base_link_name=base_link_name))

    return link_urdf_nodes(modules, urdf)
