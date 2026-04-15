"""List of robots and transform where to places them on a shared fixed base."""

from pprint import pprint
from typing import Collection, Iterable, List, NamedTuple, Type, Union

import numpy as np
from scipy.spatial.transform import Rotation

from moonbot_builder.modules.hero import (
    MhArmV1,
    MhArmV2,
    MhArmV3,
    MhCargo,
    MhWheelV1,
    MhWheelV2,
    MhWheelV3,
)

from ..modules.base import RobotModule
from ..urdf.base import Joint, Link, wrap_in_robot
from .utils import link_urdf_nodes


class MountedRobot(NamedTuple):
    """Tuple containing a RobotModule and its position in space"""

    robot: RobotModule
    xyz: np.ndarray
    quat: Rotation


def raw_urdf(
    robots: Collection[MountedRobot],
    share_start_effector: bool = False,
    base_link_name: str = "base_link",
) -> str:
    """Mounts many robot in parallel to a shared baselink in a urdf

    Args:
        robots: typically a list of robots
        share_start_effector: If true, start_effector for IK will be the shared baselink
        base_link_name:

    Returns:

    """
    urdf = ""
    base_link = Link(name=base_link_name)
    urdf += base_link.compile()

    for rob, xyz, quat in robots:
        if share_start_effector:
            rob.start_effector = base_link.name
        else:
            rob.start_effector = rob.urdf_assembler.root

        global_fix = Joint(
            f"{rob.joint_prefix}_global_fix",
            base_link.name,
            rob.urdf_assembler.root,
            xyz=xyz,
            rot=quat,
        )
        urdf += global_fix.compile() + rob.urdf_assembler.compile()

    return urdf


def parallel(
    mounted_robots: Collection[MountedRobot],
    share_start_effector: bool = False,
    base_link_name: str = "base_link",
) -> list[RobotModule]:
    """Mounts many robot in parallel to a shared baselink.

    Args:
        robots: See MountedRobot class definition.
        share_start_effector: If true, start_effector for IK will be the shared baselink
        base_link_name:

    Returns:

    """
    modules = [elem.robot for elem in mounted_robots]
    urdf = wrap_in_robot(
        raw_urdf(
            mounted_robots,
            share_start_effector=share_start_effector,
            base_link_name=base_link_name,
        )
    )

    return link_urdf_nodes(modules, urdf)


def auto_palet(
    robots: Collection[RobotModule],
) -> list[RobotModule]:
    """Automatically arranges robot on a grids

    Args:
        robots:
        base_link_name:

    Returns:

    """
    side = int(np.ceil(np.sqrt(len(robots))))
    xs, ys = np.meshgrid(np.arange(side), np.arange(side))
    coords = np.stack([ys.ravel(), xs.ravel(), np.zeros_like(ys.ravel())], axis=1)
    SPACING = 0.8
    coords = coords * SPACING

    mounted_robots: List[MountedRobot] = []
    for ind, rob in enumerate(robots):
        mted = MountedRobot(rob, xyz=coords[ind], quat=Rotation.identity())
        mounted_robots.append(mted)

    return parallel(mounted_robots)


class RobotCount(NamedTuple):
    robot: Union[
        # Type[MgArmV3],
        Type[MhArmV1],
        Type[MhWheelV1],
        Type[MhArmV2],
        Type[MhWheelV2],
        Type[MhArmV3],
        Type[MhWheelV3],
    ]
    coun: int


def family_palet(
    population: Collection[RobotCount],
    base_link_name: str = "base_link",
) -> list[RobotModule]:
    """Automatically arranges robots based on the robot count wanted for each module.

    Args:
        population:
        base_link_name:

    Returns:

    """
    SPACING = 0.8
    mounted_robots: List[MountedRobot] = []
    for ind_y, fami in enumerate(population):
        for ind_x in range(fami.coun):
            mted = MountedRobot(
                fami.robot(ind_x + 1),
                xyz=np.array([ind_x, ind_y, 0]) * SPACING,
                quat=Rotation.identity(),
            )
            mounted_robots.append(mted)

    return parallel(mounted_robots)


def mega_palet(clone_number: int = 4) -> list[RobotModule]:
    """All available robot cloned 4 times by default.

    This creates MANY robots and will be laggy. It can only work with zenoh for
    numbers above ~2.

    Args:
        clone_number: number of robots of each type to create.

    Returns:

    """
    robots: List[RobotCount] = [
        RobotCount(MhArmV1, clone_number),
        RobotCount(MhArmV2, clone_number),
        RobotCount(MhArmV3, clone_number),
        # RobotCount(MgArmV3, clone_number),
        RobotCount(MhWheelV1, clone_number),
        RobotCount(MhWheelV2, clone_number),
        RobotCount(MhWheelV3, clone_number),
    ]
    return family_palet(robots)


def g_palet(
    share_start_effector: bool = False,
    base_link_name: str = "base_link",
) -> list[RobotModule]:
    """Mounts 4 moonbot G in parallel

    Args:
        share_start_effector: If true, start_effector for IK will be the shared baselink
        base_link_name:

    Returns:

    """
    mounted = [
        MountedRobot(MgArmV3(1), np.array([0, 0, 0]), Rotation.identity()),
        MountedRobot(MgArmV3(2), np.array([0.6, 0, 0]), Rotation.identity()),
        MountedRobot(MgArmV3(3), np.array([0, 0.5, 0]), Rotation.identity()),
        MountedRobot(MgArmV3(4), np.array([0.7, 0.5, 0]), Rotation.identity()),
    ]
    return parallel(
        mounted,
        share_start_effector=share_start_effector,
        base_link_name=base_link_name,
    )


if __name__ == "__main__":
    from moonbot_builder.modules.hero import (
        MhArmV1,
        MhArmV2,
        MhArmV3,
        MhWheelV1,
        MhWheelV2,
        MhWheelV3,
    )

    mods = mega_palet(2)
    for m in mods:
        m.urdf_overide = m.urdf_assembler.jinja_path / m.urdf_assembler.jinja_file
        pprint(m)
        pprint(m.get_lvl1())
