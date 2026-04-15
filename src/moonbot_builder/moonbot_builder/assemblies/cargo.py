from typing import List

import numpy as np
from launch_ros.actions import Node
from scipy.spatial.transform import Rotation

from ..modules.base import RobotModule
from ..urdf.base import Joint, Link, wrap_in_robot
from .utils import link_urdf_nodes


def raw_urdf(
    left_wheel: RobotModule,
    right_wheel: RobotModule,
    left_arm: RobotModule,
    right_arm: RobotModule,
    cargo_box: RobotModule,
    base_link_name: str = "base_link",
) -> str:
    """
    Generate URDF for Moonbot Cargo mode:
    Central cargo box with arms and wheels attached:
    - base_link (explicit base)
    - cargo_box connects to base_link
    - left_arm connects to cargo_box, other end connects to left_wheel
    - right_arm connects to cargo_box, other end connects to right_wheel
    """
    urdf = ""
    
    base_link = Link(name=base_link_name)
    urdf += base_link.compile()
    
    cargo_to_base = Joint(
        name="cargo_to_base",
        root=base_link_name,
        leaf=cargo_box.urdf_assembler.root,
        xyz=np.array([0.0, 0.0, 0.0]),
        rot=Rotation.from_rotvec(np.array([0, np.pi/2, 0]))  
    )
    urdf += cargo_to_base.compile()
    
    urdf += cargo_box.urdf_assembler.compile()
    fixture = cargo_box.urdf_assembler.root

    fix_left_arm = Joint(
        "fix2",  
        fixture,
        left_arm.urdf_assembler.root,
        xyz=np.array([0.0, 0.0, 0.34]),
        rot=Rotation.from_rotvec(np.array([0, 0, 0]))
    )
    urdf += fix_left_arm.compile() + left_arm.urdf_assembler.compile()

    fix_left_wheel = Joint(
        "fix3",  
        left_arm.urdf_assembler.leaf,
        left_wheel.urdf_assembler.root,
        xyz=np.array([0.0, 0.0, 0.0]),
        rot=Rotation.from_rotvec(np.array([0, 0, 0]))
    )
    urdf += fix_left_wheel.compile() + left_wheel.urdf_assembler.compile()

    
    fix_right_arm = Joint(
        "fix4",  
        fixture,
        right_arm.urdf_assembler.root,
        xyz=np.array([0.0, 0.0, -0.34]),
        rot=Rotation.from_rotvec(np.array([np.pi, 0, 0]))  
    )
    urdf += fix_right_arm.compile() + right_arm.urdf_assembler.compile()

   
    fix_right_wheel = Joint(
        "fix5",  
        right_arm.urdf_assembler.leaf,
        right_wheel.urdf_assembler.root,
        xyz=np.array([0.0, 0.0, 0.0]),
        rot=Rotation.from_rotvec(np.array([0, 0, 0]))
    )
    urdf += fix_right_wheel.compile() + right_wheel.urdf_assembler.compile()

    return urdf


def cargo(
    left_wheel: RobotModule,
    right_wheel: RobotModule,
    left_arm: RobotModule,
    right_arm: RobotModule,
    cargo_box: RobotModule,
    base_link_name: str = "base_link",
) -> List[Node]:
    """Create launch nodes for cargo configuration"""
    modules = [left_wheel, right_wheel, left_arm, right_arm, cargo_box]

    urdf = wrap_in_robot(raw_urdf(
        left_wheel, right_wheel, left_arm, right_arm, cargo_box,
        base_link_name=base_link_name
    ))

    return link_urdf_nodes(modules, urdf)
