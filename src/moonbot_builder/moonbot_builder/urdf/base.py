from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from importlib.resources import as_file, files
from os.path import join
from pathlib import Path
from typing import Any, Dict, List, Literal, Optional, Tuple

import jinja2
import numpy as np
from scipy.spatial.transform import Rotation

from moonbot_builder import limb_ns

add_to_jinja_env = {
    "np": np,
    "pi": np.pi,
}


class UrdfModule(ABC):
    # loading related varibles
    #: Abstract value, must be provided by child implementation
    ros_package: str
    #: Abstract value, must be provided by child implementation
    package_relative_mesh_path: str
    #: Abstract value, must be provided by child implementation
    # Fallsback to package_relative_mesh_path if not provided
    package_relative_collision_mesh_path: Optional[str] = None
    #: Abstract value, must be provided by child implementation
    jinja_file: str
    #: Abstract value, must be provided by child implementation
    mesh_extension: str

    # compilation related varibles
    #: Semi-Abstract value, must be completed by child implementation
    name: str
    #: Semi-Abstract value, must be completed by child implementation
    additional_joints: List[str]
    #: Abstract value, must be provided by child implementation
    root: str
    #: defaults to f"{limb_ns(limb_number)}_"
    prefix: str

    def __init__(self, limb_number: int):
        self.name = ""
        self.additional_joints = []
        self.pkg_path = Path(files(self.ros_package))
        self.jinja_path = self.pkg_path / "jinja"
        self.mesh_path = (
            self.pkg_path / self.package_relative_mesh_path
        ).as_uri() + "/"
        # if no collision mesh is provided, use the same as the visual mesh
        if self.package_relative_collision_mesh_path is None:
            self.package_relative_collision_mesh_path = self.package_relative_mesh_path

        self.collision_mesh_path = (
            self.pkg_path / self.package_relative_collision_mesh_path
        ).as_uri() + "/"
        self.prefix = f"{limb_ns(limb_number)}_"

    @abstractmethod
    def compile_kwarg(self) -> Dict[str, Any]:
        return {
            "prefix": self.prefix,
            "mesh_extension": self.mesh_extension,
            "mesh_path": self.mesh_path,
            "collision_mesh_path": self.collision_mesh_path,
            "root": self.root,
        }

    def compile(self) -> str:
        env = jinja2.Environment(
            loader=jinja2.FileSystemLoader(self.jinja_path), autoescape=False
        )
        env.globals.update(add_to_jinja_env)
        tempate = env.get_template(self.jinja_file)
        kwargs = self.compile_kwarg()
        return tempate.render(**kwargs)

    def verify(self):
        """Checks that all variables are defined, if not, raises error"""
        self.ros_package
        self.package_relative_mesh_path
        self.jinja_path
        self.jinja_file
        self.mesh_extension
        self.name
        self.root
        self.prefix


class Arm(UrdfModule):
    #: end-effector
    leaf: str

    def __init__(self, limb_number: int):
        super().__init__(limb_number)
        self.name += f"arm"
        self.root = f"{self.prefix}in"
        self.leaf = f"{self.prefix}out"

    def compile_kwarg(self) -> Dict[str, Any]:
        kwargs = super().compile_kwarg()
        new = {f"leaf": self.leaf}
        kwargs.update(new)
        return kwargs

    def verify(self):
        super().verify()
        self.leaf


class Wheel(UrdfModule):
    #: second mounting point
    leaf: str
    #: names of the tire joints (necessary because the motion stack will not detect them automatically, this info must be present at launch-time)
    #: Abstract value, must be provided by child implementation
    tire_joints: List[str]

    def __init__(self, limb_number: int):
        super().__init__(limb_number)
        self.name += f"wheel"
        self.tire_joints = []
        self.prefix = f"wheel{limb_number}_"
        self.root = f"{self.prefix}in"
        self.leaf = f"{self.prefix}out"

    def compile_kwarg(self) -> Dict[str, Any]:
        kwargs = super().compile_kwarg()
        new = {f"leaf": self.leaf}
        kwargs.update(new)
        return kwargs

    def verify(self):
        super().verify()
        self.leaf
        self.tire_joints: List[str]


@dataclass
class Link:
    name: str

    def compile(self) -> str:
        return f"""<link
name="{ self.name }">
</link>"""


@dataclass
class Joint:
    name: str
    root: str
    leaf: str
    jtype: Literal["fixed", "revolute", "continuous", "linear"] = "fixed"
    xyz: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rot: Rotation = field(default_factory=Rotation.identity)

    def compile(self):
        rpy = self.rot.as_euler("XYZ", degrees=False)
        return f"""\n
<joint name="{ self.name }" type="{self.jtype}">
  <origin rpy="{rpy[0]} {rpy[1]} {rpy[2]}" xyz="{self.xyz[0]} {self.xyz[1]} {self.xyz[2]}"></origin>
  <parent link="{self.root}"></parent>
  <child link="{self.leaf}"></child>
</joint>\n
    """


def wrap_in_robot(xml: str, robot_name: str = "my_robot"):
    return rf"""<?xml version="1.0" encoding="utf-8"?>
<robot
name="{robot_name}">
{xml}
</robot>
"""


#
# a=WheelV2(3)
# w=ArmV2(1)
# j=Joint(name="test_fixed", root = a.leaf, leaf=w.root)
#
# print(a.compile() + w.compile() + j.compile())
