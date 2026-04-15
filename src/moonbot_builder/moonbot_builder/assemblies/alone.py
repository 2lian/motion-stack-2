"""Assembles one arm or arms one after the other in a chain"""

from copy import deepcopy
from typing import List, Union

import numpy as np
from launch_ros.actions import Node
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
        module.down_from, module.up_to = 1, 1
        module.start_effector = module.urdf_assembler.root
        urdf += fix.compile() + module.urdf_assembler.compile()
        fixture = module.urdf_assembler.leaf

    return urdf


def alone(
    arm: Union[RobotModule, List[RobotModule]],
    base_link_name: str = "base_link",
    xyz: np.ndarray = np.zeros(3),
) -> List[Node]:
    """Assembles one arm or arms one after the other in a chain"""
    modules: List[RobotModule] = [arm] if not isinstance(arm, list) else arm
    ik_module = deepcopy(modules[0])

    urdf = wrap_in_robot(raw_urdf(modules, base_link_name=base_link_name, xyz=xyz))
    relays = []

    for count, module in enumerate(modules):
        if count == 0:
            continue

        relays.append(
            Node(
                package="topic_tools",
                executable="relay",
                name="read_relay",
                namespace=module.namespace,
                parameters=[
                    # (intside node, outside node),
                    {
                        "input_topic": "joint_read",
                        "output_topic": f"/{ik_module.namespace}/joint_read",
                    }
                ],
            )
        )

        relays.append(
            Node(
                package="topic_tools",
                executable="relay",
                name="set_relay",
                namespace=module.namespace,
                parameters=[
                    # (intside node, outside node),
                    {
                        "output_topic": "joint_set",
                        "input_topic": f"/{ik_module.namespace}/joint_set",
                    }
                ],
            )
        )

    ik_module.start_effector = modules[0].start_effector
    # ik_module.end_effector = modules[-1].end_effector
    ik_module.end_effector = f"{modules[-1].joint_prefix}out_straight"
    ik_module.down_from, ik_module.up_to = 2, 2

    modules.append(ik_module)

    return link_urdf_nodes(modules, urdf) + relays
