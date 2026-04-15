from typing import Any, Dict, List

from .base import Arm, UrdfModule, Wheel


class HeroRobot(UrdfModule):
    ros_package = "hero_assets"

    def __init__(self, limb_number: int):
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
    package_relative_mesh_path = "meshes/hero_7dof_arm/collada/"
    package_relative_collision_mesh_path = "meshes/hero_7dof_arm/collision/"
    jinja_file = "hero_arm_v1.jinja.urdf"
    mesh_extension = "dae"

    def __init__(self, limb_number: int, reverse: bool = False):
        super().__init__(limb_number, reverse)
        self.name += f"v1-{limb_number}"


class HeroArmV2(HeroArm):
    package_relative_mesh_path = "meshes/hero_7dof_arm_v2/raw/collada/"
    package_relative_collision_mesh_path = "meshes/hero_7dof_arm_v2/collision/"
    jinja_file = "hero_arm_v2.jinja.urdf"
    mesh_extension = "dae"

    def __init__(self, limb_number: int, reverse: bool = False):
        super().__init__(limb_number, reverse)
        self.name += f"v2-{limb_number}"


class HeroArmV3(HeroArm):
    package_relative_mesh_path = "meshes/hero_7dof_arm_v3/"
    package_relative_collision_mesh_path = "meshes/hero_7dof_arm_v3/"
    jinja_file = "hero_arm_v3.jinja.urdf"
    mesh_extension = "STL"

    def __init__(self, limb_number: int, reverse: bool = False):
        super().__init__(limb_number, reverse)
        self.name += f"v3-{limb_number}"


class HeroWheel(Wheel, HeroRobot): ...


class HeroWheelV1(HeroWheel):
    package_relative_mesh_path = "meshes/hero_wheel_module/collada/"
    package_relative_collision_mesh_path = "meshes/hero_wheel_module/collision/"
    jinja_file = "hero_wheel_v1.jinja.urdf"
    mesh_extension = "dae"

    def __init__(self, limb_number: int):
        super().__init__(limb_number)
        self.name += f"v1-{limb_number}"
        self.tire_joints += [f"{self.prefix}left_joint", f"{self.prefix}right_joint"]
        self.additional_joints += self.tire_joints


class HeroWheelV2(HeroWheel):
    package_relative_mesh_path = "meshes/hero_wheel_module_v2/raw/collada/"
    package_relative_collision_mesh_path = "meshes/hero_wheel_module_v2/collision/"
    jinja_file = "hero_wheel_v2.jinja.urdf"
    mesh_extension = "dae"

    def __init__(self, limb_number: int):
        super().__init__(limb_number)
        self.name += f"v2-{limb_number}"
        self.tire_joints += [f"{self.prefix}left_joint", f"{self.prefix}right_joint"]
        self.additional_joints += self.tire_joints


class HeroCargo(HeroRobot):
    package_relative_mesh_path = "meshes/hero_cargo/collada/"
    package_relative_collision_mesh_path = "meshes/hero_cargo/collision/"
    jinja_file = "hero_cargo.jinja.urdf"
    mesh_extension = "dae"

    def __init__(self, limb_number: int):
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
    package_relative_mesh_path = "meshes/sled/collada/"
    package_relative_collision_mesh_path = "meshes/sled/collision/"
    jinja_file = "hero_cargo_v2.jinja.urdf"
    mesh_extension = "dae"

    def __init__(self, limb_number: int):
        super().__init__(limb_number)
        self.name += f"cargo_v2-{limb_number}"
        self.prefix = "cargo_"
        self.root = f"{self.prefix}base_link"
        self.leaf = f"{self.prefix}base_link"
        self.namespace = "cargo"

    def compile_kwarg(self) -> Dict[str, Any]:
        default = super().compile_kwarg()
        default.update({
            "ros_package": self.ros_package,
            "package_relative_mesh_path": self.package_relative_mesh_path,
            "package_relative_collision_mesh_path": self.package_relative_collision_mesh_path,
        })
        return default


class HeroBody(HeroRobot):
    package_relative_mesh_path = "meshes/hero_body/"
    package_relative_collision_mesh_path = "meshes/hero_body/"
    jinja_file = "hero_body.jinja.urdf"
    mesh_extension = "stl"

    def __init__(self, limb_number):
        super().__init__(limb_number)
        self.name += f"body-{limb_number}"
        self.root = f"tri_body"
        self.leaf = f"tri_body"
