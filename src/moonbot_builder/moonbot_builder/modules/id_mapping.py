from typing import Optional

from moonbot_launcher.modules.base import Oli, RobotModule, collapse_limb_id, parse_id

from .gusta import MgArmV3
from .hero import MhArmV1, MhArmV2, MhArmV3, MhWheelV0, MhWheelV1, MhWheelV2, MhWheelV3


def id2robot(id: Oli = None) -> Optional[RobotModule]:
    id = collapse_limb_id(id)
    parsed = parse_id(id)
    if parsed is None:
        return None
    key = parsed[0:3]
    number = parsed[3]
    module_obj = {
        ("hero", "arm", 1): MhArmV1,
        ("hero", "arm", 2): MhArmV2,
        ("hero", "arm", 3): MhArmV3,
        ("hero", "wheel", 0): MhWheelV0,
        ("hero", "wheel", 1): MhWheelV1,
        ("hero", "wheel", 2): MhWheelV2,
        ("hero", "wheel", 3): MhWheelV3,
        ("gustavo", "arm", 3): MgArmV3,
    }[key]
    return module_obj(number)


def id2namespace(id: Oli = None) -> str:
    robot = id2robot(id)
    if robot is None:
        return ""
    return robot.namespace


def index2robot(limb_index: int):
    if 0 < limb_index < 10:
        return MhArmV1(limb_index - 0)
    if 10 < limb_index < 20:
        return MhWheelV1(limb_index - 10)
    if 50 < limb_index < 60:
        return MgArmV3(limb_index - 50)
    if 200 < limb_index < 210:
        return MhArmV2(limb_index - 200)
    if 210 < limb_index < 220:
        return MhWheelV2(limb_index - 210)
    if 300 < limb_index < 310:
        return MhArmV3(limb_index - 300)
    if 310 < limb_index < 320:
        return MhWheelV3(limb_index - 310)
    raise ValueError(
        f"Robot module could not be found for the given limb index {limb_index}"
    )
