from importlib.resources import as_file, files
from pprint import pprint
from typing import Any, Dict, cast

from .. import urdf
from .base import Lvl1Node, ModuleType, RobotModule


class HeroModule(RobotModule):
    brand = "hero"
    package = "hero_ros2"

    def __init__(self, module: ModuleType, version: int, number: int) -> None:
        self.number = number
        super().__init__(self.brand, module, version, number)
        self.up_to = 2
        self.executable = ["python3", "-m", "ms_pyzeros_bridge.lvl1_exec"]

    def get_lvl1(self) -> list[Lvl1Node]:
        deflt = super().get_lvl1()
        deflt[0].ms_params.ignore_limits = True
        return deflt


class MhArm(HeroModule):
    module = "arm"

    def __init__(self, version: int, number: int) -> None:
        self.number = number
        super().__init__(self.module, version, number)
        self.add_joints += self.gripper_joints()

    def gripper_joints(self):
        return [
            f"{self.joint_prefix}grip1",
            f"{self.joint_prefix}grip2",
        ]


class MhArmV1(MhArm):
    version = 1

    def __init__(self, number: int, reverse: bool = False) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroArmV1(
            limb_number=self.limb_index, reverse=reverse
        )
        super().__init__(self.version, number)

    @property
    def limb_index(self) -> int:
        return self.number


class MhArmV2(MhArm):
    version = 2

    def __init__(self, number: int, reverse: bool = False) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroArmV2(
            limb_number=self.limb_index, reverse=reverse
        )
        super().__init__(self.version, number)

    @property
    def limb_index(self) -> int:
        return 200 + self.number


class MhArmV3(MhArm):
    version = 3

    def __init__(self, number: int, reverse: bool = False) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroArmV3(
            limb_number=self.limb_index, reverse=reverse
        )
        super().__init__(self.version, number)

    @property
    def limb_index(self) -> int:
        return 300 + self.number


class MhWheel(HeroModule):
    module = "wheel"

    def __init__(self, version: int, number: int) -> None:
        self.number = number
        super().__init__(self.module, version, number)
        self.add_joints += self.wheel_joints()

    def wheel_joints(self):
        return [
            f"{self.joint_prefix}left_joint",
            f"{self.joint_prefix}right_joint",
        ]

    def get_lvl1(self) -> list[Lvl1Node]:
        deflt = super().get_lvl1()
        deflt[0].ms_params.ignore_limits = True
        deflt[0].ms_params.start_effector_name = deflt[0].ms_params.end_effector_name
        return deflt


class MhWheelV0(MhWheel):
    version = 0

    def __init__(self, number: int) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroWheelV1(limb_number=self.limb_index)
        super().__init__(self.version, number)

    @property
    def limb_index(self) -> int:
        return 10 + self.number


class MhWheelV1(MhWheel):
    version = 1

    def __init__(self, number: int) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroWheelV1(limb_number=self.limb_index)
        super().__init__(self.version, number)
        # self.add_joints += [self.joint_prefix + "grasp_joint"]

    @property
    def limb_index(self) -> int:
        return 10 + self.number


class MhWheelV2(MhWheel):
    version = 2

    def __init__(self, number: int) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroWheelV2(limb_number=self.limb_index)
        super().__init__(self.version, number)
        self.add_joints += [
            f"{self.joint_prefix}left_flap",
            f"{self.joint_prefix}right_flap",
        ]

    @property
    def limb_index(self) -> int:
        return 10 + 200 + self.number


class MhWheelV3(MhWheel):
    version = 3

    def __init__(self, number: int) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroWheelV2(limb_number=self.limb_index)
        super().__init__(self.version, number)
        self.add_joints += [
            f"{self.joint_prefix}left_flap",
            f"{self.joint_prefix}right_flap",
        ]

    @property
    def limb_index(self) -> int:
        return 10 + 300 + self.number


class MhCargo(HeroModule):
    module = "cargo"

    def __init__(self, number: int) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroCargo(limb_number=self.limb_index)
        super().__init__(self.module, 1, number)
        self.namespace = f"cargo{self.number:02d}"

    @property
    def limb_index(self) -> int:
        return 500 + self.number

    def get_nodes(self):
        """Override to return empty list - cargo box is structural only, no nodes needed"""
        print(self.friendly_display() + " | structural component (no nodes)")
        return []


class MhCargoV2(HeroModule):
    """CargoV2 uses sled mesh instead of cargo_body"""

    module = "cargo"

    def __init__(self, number: int) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroCargoV2(limb_number=self.limb_index)
        super().__init__(self.module, 2, number)
        self.namespace = f"cargo{self.number:02d}"

    @property
    def limb_index(self) -> int:
        return 500 + 100 + self.number

    def get_nodes(self):
        """Override to return empty list - cargo box is structural only, no nodes needed"""
        print(self.friendly_display() + " | structural component (no nodes)")
        return []


class MhBody(HeroModule):
    module = "body"

    def __init__(self, number: int = 1) -> None:
        self.number = number
        self.urdf_assembler = urdf.HeroBody(limb_number=self.limb_index)
        super().__init__(self.module, 1, number)

    @property
    def limb_index(self) -> int:
        return 10 + 500 + self.number

    def _node_args(self) -> Dict[int, Dict[str, Any]]:
        args = dict()
        return args


if __name__ == "__main__":
    for m in [
        MhArmV1(1),
        MhArmV2(1),
        MhArmV3(1),
        MhWheelV1(1),
        MhWheelV2(1),
        MhWheelV3(1),
    ]:
        m = cast(RobotModule, m)
        np = m.get_lvl1()[0]
        np.ms_params.urdf = m.urdf_assembler.jinja_path / m.urdf_assembler.jinja_file
        pprint(np)
