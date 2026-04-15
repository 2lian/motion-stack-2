from typing import Any, Dict

from .. import urdf
from .base import ModuleType, RobotModule


class RealmanModule(RobotModule):
    brand = "realman"
    package = "realman_interface"

    def __init__(self, module: ModuleType, version: int, number: int) -> None:
        self.number = number
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

    def global_params(self) -> Dict:
        par = super().global_params()
        par.update(
            {
                "ignore_limits": False,
                "speed_mode": False,
            }
        )
        return par


class RealmanArm(RealmanModule):
    version = 1
    module = "arm"
    number = 75

    def __init__(self) -> None:
        self.urdf_assembler = urdf.RealmanArm(limb_number=self.limb_index)
        super().__init__(self.module, self.version, self.number)

    @property
    def limb_index(self) -> int:
        return self.number
