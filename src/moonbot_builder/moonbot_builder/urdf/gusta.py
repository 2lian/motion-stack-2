from typing import Any, Dict, List

from .base import Arm, UrdfModule, Wheel


class GustaRobot(UrdfModule):
    ros_package = "moonbotg_4dof_rppr_ultraslim"

    def __init__(self, limb_number: int):
        super().__init__(limb_number)
        self.name += "mg"
        self.joint_list: List[str] = []

    def compile_kwarg(self) -> Dict[str, Any]:
        default = super().compile_kwarg()
        default.update(
            {
                "joint_list": self.joint_list,
            }
        )
        return default

class GustaArm(Arm, GustaRobot):
    def __init__(self, limb_number: int, reverse: bool = False):
        super().__init__(limb_number)
        self.joint_list: List[str] = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
        ]
        if reverse:
            self.joint_list.reverse()

class GustaArmV3(GustaArm):
    # package_relative_mesh_path = "meshes/moonbotg_v3/"
    # jinja_file = "g_armv3.jinja.urdf"
    package_relative_mesh_path = "meshes/moonbotg_v3_updated/"
    jinja_file = "g_armv3_updated.jinja.urdf"
    mesh_extension = "STL"
    _gripper_joint_type: str

    def __init__(
        self, limb_number: int, reverse: bool = False, enable_gripper: bool = False
    ):
        super().__init__(limb_number, reverse=reverse)
        self.name += f"v2-{limb_number}"
        if enable_gripper:
            self._gripper_joint_type = "prismatic"
        else:
            self._gripper_joint_type = "fixed"

    def compile_kwarg(self) -> Dict[str, Any]:
        kwargs = super().compile_kwarg()
        new = {f"grip_joint_type": self._gripper_joint_type}
        kwargs.update(new)
        return kwargs
