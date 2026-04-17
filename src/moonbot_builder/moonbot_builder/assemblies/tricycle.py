from typing import List, Optional

import numpy as np
from scipy.spatial.transform import Rotation

from ..modules.base import RobotModule
from ..urdf.base import Joint, Link, wrap_in_robot
from .utils import link_urdf_nodes


def raw_urdf_tricycle(
    front_wheel: RobotModule,
    left_wheel: RobotModule,
    right_wheel: RobotModule,
    front_arm: RobotModule,
    left_arm: RobotModule,
    right_arm: RobotModule,
    central_body: RobotModule,
    manipulator: Optional[RobotModule] = None,
    base_link_name: str = "base_link_tricycle",
) -> str:

    urdf = ""

    base_link = Link(name=base_link_name)
    urdf += base_link.compile()

    central_to_base = Joint(
        name="central_to_base",
        root=base_link_name,
        leaf=central_body.urdf_assembler.root,
        xyz=np.array([0.0, 0.0, 0.5]),
        rot=Rotation.identity(),
    )
    urdf += central_to_base.compile()

    urdf += central_body.urdf_assembler.compile()
    fixture = central_body.urdf_assembler.root

    body_radius = 0.200

    def arm_pose(
        yaw_deg: float,
        angle_deg: float,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
    ):

        yaw = np.deg2rad(yaw_deg)
        angle = np.deg2rad(angle_deg)
        roll = np.deg2rad(roll_deg)
        pitch = np.deg2rad(pitch_deg)

        xyz = np.array(
            [
                body_radius * np.cos(angle),
                body_radius * np.sin(angle),
                0.0,
            ]
        )

        r_x = Rotation.from_rotvec(np.array([roll, 0.0, 0.0]))
        r_y = Rotation.from_rotvec(np.array([0.0, pitch, 0.0]))
        r_z = Rotation.from_rotvec(np.array([0.0, 0.0, yaw]))

        rot = r_y * r_x * r_z

        return xyz, rot

    front_xyz, front_rot = arm_pose(
        yaw_deg=180.0,
        angle_deg=0.0,
        roll_deg=90.0,
        pitch_deg=90.0,
    )
    fix_front_arm = Joint(
        "fix_front_arm",
        fixture,
        front_arm.urdf_assembler.root,
        xyz=front_xyz,
        rot=front_rot,
    )
    urdf += fix_front_arm.compile() + front_arm.urdf_assembler.compile()

    fix_front_wheel = Joint(
        "fix_front_wheel",
        front_arm.urdf_assembler.leaf,
        front_wheel.urdf_assembler.root,
        xyz=np.zeros(3),
        rot=Rotation.identity(),
    )
    urdf += fix_front_wheel.compile() + front_wheel.urdf_assembler.compile()
    # print(urdf)

    right_xyz, right_rot = arm_pose(
        yaw_deg=-60.0,
        angle_deg=120.0,
        roll_deg=90.0,
        pitch_deg=90.0,
    )

    fix_right_arm = Joint(
        "fix_right_arm",
        fixture,
        right_arm.urdf_assembler.root,
        xyz=right_xyz,
        rot=right_rot,
    )
    urdf += fix_right_arm.compile() + right_arm.urdf_assembler.compile()

    fix_right_wheel = Joint(
        "fix_right_wheel",
        right_arm.urdf_assembler.leaf,
        right_wheel.urdf_assembler.root,
        xyz=np.zeros(3),
        rot=Rotation.identity(),
    )
    urdf += fix_right_wheel.compile() + right_wheel.urdf_assembler.compile()

    left_xyz, left_rot = arm_pose(
        yaw_deg=60.0,
        angle_deg=240.0,
        roll_deg=90.0,
        pitch_deg=90.0,
    )
    fix_left_arm = Joint(
        "fix_left_arm",
        fixture,
        left_arm.urdf_assembler.root,
        xyz=left_xyz,
        rot=left_rot,
    )
    urdf += fix_left_arm.compile() + left_arm.urdf_assembler.compile()

    front_arm.start_effector = ""
    left_arm.start_effector = ""
    right_arm.start_effector = ""

    fix_left_wheel = Joint(
        "fix_left_wheel",
        left_arm.urdf_assembler.leaf,
        left_wheel.urdf_assembler.root,
        xyz=np.zeros(3),
        rot=Rotation.identity(),
    )
    urdf += fix_left_wheel.compile() + left_wheel.urdf_assembler.compile()

    if manipulator is not None:
        _, manip_rot = arm_pose(
            yaw_deg=60.0,
            angle_deg=0.0,
            roll_deg=0.0,
            pitch_deg=0.0,
        )
        manip_xyz = np.array([0.0, 0.0, 0.13])
        fix_manip = Joint(
            "fix_manip",
            fixture,
            manipulator.urdf_assembler.root,
            xyz=manip_xyz,
            rot=manip_rot,
        )
        urdf += fix_manip.compile() + manipulator.urdf_assembler.compile()
        manipulator.start_effector = ""
        manipulator.end_effector = f"{manipulator.joint_prefix}out_straight"

    return urdf


def tricycle(
    front_wheel: RobotModule,
    left_wheel: RobotModule,
    right_wheel: RobotModule,
    front_arm: RobotModule,
    left_arm: RobotModule,
    right_arm: RobotModule,
    central_body: RobotModule,
    manipulator: Optional[RobotModule] = None,
    base_link_name: str = "base_link_tricycle",
) -> List:

    modules = [
        front_wheel,
        left_wheel,
        right_wheel,
        front_arm,
        left_arm,
        right_arm,
        central_body,
    ]

    if manipulator is not None:
        modules.append(manipulator)

    urdf = wrap_in_robot(
        raw_urdf_tricycle(
            *modules,
            base_link_name=base_link_name,
        )
    )

    return link_urdf_nodes(modules, urdf)
