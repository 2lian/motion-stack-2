from enum import Enum
from pathlib import Path
from typing import Any, Dict, List

from .base import Arm, UrdfModule, Wheel


class MESH_TYPE(Enum):
    DAE = 0
    RAW = 1
    OPTI = 2


_MESH_TYPE_DIR = {
    MESH_TYPE.DAE: Path("collada"),
    MESH_TYPE.RAW: Path("raw"),
    MESH_TYPE.OPTI: Path("opti"),
}

_MESH_TYPE_EXT = {
    MESH_TYPE.DAE: "dae",
    MESH_TYPE.RAW: "STL",
    MESH_TYPE.OPTI: "STL",
}


DEFAULT_MESH_TYPE: MESH_TYPE = MESH_TYPE.DAE


class HeroRobot(UrdfModule):
    ros_package = "hero_assets_py"
    mesh_base_path: str
    mesh_type_extensions: dict = _MESH_TYPE_EXT
    mesh_type_fallback: dict[MESH_TYPE, MESH_TYPE] = {}

    def __init__(self, limb_number: int, mesh_type: MESH_TYPE | None = None):
        mesh_type = getattr(self, "_mesh_type", mesh_type)
        if mesh_type is None:
            mesh_type = DEFAULT_MESH_TYPE
        mesh_type = self.mesh_type_fallback.get(mesh_type, mesh_type)
        self.mesh_type_dir = _MESH_TYPE_DIR[mesh_type]
        self.package_relative_mesh_path = str(
            Path(self.mesh_base_path) / self.mesh_type_dir
        )
        self.mesh_extension = self.mesh_type_extensions[mesh_type]

        super().__init__(limb_number)
        self.name += "mh"
        self.joint_list: List[str] = []
        self.gripper_list: List[str] = []

    def compile_kwarg(self) -> Dict[str, Any]:
        default = super().compile_kwarg()
        default.update(
            {
                "joint_list": self.joint_list,
                "gripper_list": self.gripper_list,
            }
        )
        return default


class HeroArm(Arm, HeroRobot):
    def __init__(self, limb_number: int, reverse: bool = False):
        super().__init__(limb_number)
        self.joint_list: List[str] = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]
        self.gripper_list: List[str] = ["grip1", "grip2"]
        if reverse:
            self.joint_list.reverse()


class HeroArmV1(HeroArm):
    mesh_base_path = "meshes/hero_7dof_arm"
    package_relative_collision_mesh_path = "meshes/hero_7dof_arm/collision/"
    jinja_file = "hero_arm_v1.jinja.urdf"

    def __init__(self, limb_number: int, reverse: bool = False, mesh_type: MESH_TYPE | None = None):
        self._mesh_type = mesh_type
        super().__init__(limb_number, reverse)
        self.name += f"v1-{limb_number}"


class HeroArmV2(HeroArm):
    mesh_base_path = "meshes/hero_7dof_arm_v2"
    package_relative_collision_mesh_path = "meshes/hero_7dof_arm_v2/collision/"
    jinja_file = "hero_arm_v2.jinja.urdf"

    def __init__(self, limb_number: int, reverse: bool = False, mesh_type: MESH_TYPE | None = None):
        self._mesh_type = mesh_type
        super().__init__(limb_number, reverse)
        self.name += f"v2-{limb_number}"


class HeroArmV3(HeroArm):
    mesh_base_path = "meshes/hero_7dof_arm_v3"
    jinja_file = "hero_arm_v3.jinja.urdf"
    mesh_type_fallback = {MESH_TYPE.DAE: MESH_TYPE.RAW}

    def __init__(self, limb_number: int, reverse: bool = False, mesh_type: MESH_TYPE | None = None):
        self._mesh_type = mesh_type
        super().__init__(limb_number, reverse)
        self.name += f"v3-{limb_number}"


class HeroWheel(Wheel, HeroRobot): ...


class HeroWheelV1(HeroWheel):
    mesh_base_path = "meshes/hero_wheel_module"
    package_relative_collision_mesh_path = "meshes/hero_wheel_module/collision/"
    jinja_file = "hero_wheel_v1.jinja.urdf"

    def __init__(self, limb_number: int, mesh_type: MESH_TYPE | None = None):
        self._mesh_type = mesh_type
        super().__init__(limb_number)
        self.name += f"v1-{limb_number}"
        self.tire_joints += [f"{self.prefix}left_joint", f"{self.prefix}right_joint"]
        self.additional_joints += self.tire_joints


class HeroWheelV2(HeroWheel):
    mesh_base_path = "meshes/hero_wheel_module_v2"
    package_relative_collision_mesh_path = "meshes/hero_wheel_module_v2/collision/"
    jinja_file = "hero_wheel_v2.jinja.urdf"

    def __init__(self, limb_number: int, mesh_type: MESH_TYPE | None = None):
        self._mesh_type = mesh_type
        super().__init__(limb_number)
        self.name += f"v2-{limb_number}"
        self.tire_joints += [f"{self.prefix}left_joint", f"{self.prefix}right_joint"]
        self.additional_joints += self.tire_joints


class HeroCargo(HeroRobot):
    mesh_base_path = "meshes/hero_cargo"
    package_relative_collision_mesh_path = "meshes/hero_cargo/collision/"
    jinja_file = "hero_cargo.jinja.urdf"
    mesh_type_fallback = {MESH_TYPE.OPTI: MESH_TYPE.RAW}
    mesh_type_extensions = {
        MESH_TYPE.DAE: "dae",
        MESH_TYPE.RAW: "stl",
        MESH_TYPE.OPTI: "stl",
    }

    def __init__(self, limb_number: int, mesh_type: MESH_TYPE | None = None):
        self._mesh_type = mesh_type
        super().__init__(limb_number)
        self.name += f"cargo-{limb_number}"
        self.prefix = "cargo_"
        self.root = f"{self.prefix}base_link"
        self.leaf = f"{self.prefix}base_link"
        self.namespace = "cargo"

    def compile_kwarg(self) -> Dict[str, Any]:
        default = super().compile_kwarg()
        default.update(
            {
                "ros_package": self.ros_package,
                "package_relative_mesh_path": self.package_relative_mesh_path,
                "package_relative_collision_mesh_path": self.package_relative_collision_mesh_path,
            }
        )
        return default


class HeroCargoV2(HeroRobot):
    """CargoV2 uses sled instead of cargo_body"""

    mesh_base_path = "meshes/sled"
    package_relative_collision_mesh_path = "meshes/sled/collision/"
    jinja_file = "hero_cargo_v2.jinja.urdf"
    mesh_type_fallback = {MESH_TYPE.OPTI: MESH_TYPE.RAW}

    def __init__(self, limb_number: int, mesh_type: MESH_TYPE | None = None):
        self._mesh_type = mesh_type
        super().__init__(limb_number)
        self.name += f"cargo_v2-{limb_number}"
        self.prefix = "cargo_"
        self.root = f"{self.prefix}base_link"
        self.leaf = f"{self.prefix}base_link"
        self.namespace = "cargo"

    def compile_kwarg(self) -> Dict[str, Any]:
        default = super().compile_kwarg()
        default.update(
            {
                "ros_package": self.ros_package,
                "package_relative_mesh_path": self.package_relative_mesh_path,
                "package_relative_collision_mesh_path": self.package_relative_collision_mesh_path,
            }
        )
        return default


class HeroBody(HeroRobot):
    mesh_base_path = "meshes/hero_body"
    package_relative_collision_mesh_path = "meshes/hero_body/raw/"
    jinja_file = "hero_body.jinja.urdf"
    mesh_type_fallback = {MESH_TYPE.DAE: MESH_TYPE.RAW}
    mesh_type_extensions = {
        MESH_TYPE.DAE: "dae",
        MESH_TYPE.RAW: "stl",
        MESH_TYPE.OPTI: "stl",
    }

    def __init__(self, limb_number, mesh_type: MESH_TYPE | None = None):
        self._mesh_type = mesh_type
        super().__init__(limb_number)
        self.name += f"body-{limb_number}"
        self.root = f"tri_body"
        self.leaf = f"tri_body"
