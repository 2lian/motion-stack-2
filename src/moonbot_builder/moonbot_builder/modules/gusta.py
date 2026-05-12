from .. import urdf
from .base import Lvl1Node, ModuleType, RobotModule


class GustaModule(RobotModule):
    brand = "gustavo"

    def __init__(self, module: ModuleType, version: int, number: int) -> None:
        self.number = number
        super().__init__(self.brand, module, version, number)
        self.up_to = 2
        self.executable = ["python3", "-m", "ms_pyzeros_bridge.lvl1_exec"]

    def get_lvl1(self) -> list[Lvl1Node]:
        deflt = super().get_lvl1()
        deflt[0].ms_params.ignore_limits = True
        return deflt


class MgArm(GustaModule):
    module = "arm"

    def __init__(self, version: int, number: int) -> None:
        self.number = number
        super().__init__(self.module, version, number)

    def gripper_joints(self):
        return [
            f"{self.joint_prefix}grip1",
            f"{self.joint_prefix}grip2",
            f"{self.joint_prefix}grip1bis",
            f"{self.joint_prefix}grip2bis",
        ]


class MgArmV3(MgArm):
    version = 3

    def __init__(
        self, number: int, reverse: bool = False, enable_gripper: bool = False
    ) -> None:
        self.number = number
        self.urdf_assembler = urdf.GustaArmV3(
            limb_number=self.limb_index,
            enable_gripper=enable_gripper,
            reverse=reverse,
        )
        super().__init__(self.version, number)
        if enable_gripper:
            self.add_joints += self.gripper_joints()

    @property
    def limb_index(self) -> int:
        return 50 + self.number
