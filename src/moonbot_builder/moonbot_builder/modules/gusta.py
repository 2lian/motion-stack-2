from copy import deepcopy
from typing import Any, Dict, List, Optional

from moonbot_launcher.modules.hero import MhArm

from .. import urdf
from .base import ModuleType, RobotModule


class GustaModule(RobotModule):
    brand = "gustavo"
    package = "moonbotg_4dof_rppr_ultraslim"

    def __init__(self, module: ModuleType, version: int, number: int) -> None:
        super().__init__(self.brand, module, version, number)
        self.up_to = 2

    def _node_args(self) -> Dict[int, Dict[str, Any]]:
        args = super()._node_args()
        args[1].update(
            {
                "package": self.package,
            }
        )
        return args

    def lvl_params(self) -> Dict:
        overall_params = super().lvl_params()
        overall_params[1].update({"mgsim_joint_remapping": not self.simu_mode})
        return overall_params


class MgArm(GustaModule):
    module = "arm"

    def __init__(self, version: int, number: int) -> None:
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
        super().__init__(self.version, number)
        self.urdf_assembler = urdf.GustaArmV3(
            limb_number=self.limb_index, enable_gripper=enable_gripper, reverse=reverse
        )
        if enable_gripper:
            self.add_joints += self.gripper_joints()

    def global_params(self) -> Dict:
        par = super().global_params()
        par.update(
            {
                "ignore_limits": False,
            }
        )
        return par

    # @staticmethod
    # def gripper_joints(limb_index: int):
    #     return MhArm.gripper_joints(limb_index)
    #     # return self.gripper_joints(limb_index)

    # def _gripper_joints(self):
    #     return self.gripper_joints(self.limb_index)

    @property
    def limb_index(self) -> int:
        return 50 + self.number
