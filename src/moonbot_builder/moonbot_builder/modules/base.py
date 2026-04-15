from abc import ABC, abstractmethod
from copy import deepcopy
from dataclasses import dataclass
from os import getenv
from subprocess import Popen
from typing import (
    Any,
    Dict,
    Final,
    List,
    Literal,
    Mapping,
    NamedTuple,
    Optional,
    Set,
    Tuple,
    Union,
)

import numpy as np
from colorama import Fore
from motion_stack.lvl1.core import Lvl1Param

from moonbot_builder import limb_ns, urdf

#: optional limb id, if None, the environment variable will be used and parsed
Oli = Optional[str]
RobotBrands = Literal["hero", "gustavo", "realman"]
ModuleType = Literal["arm", "wheel", "body"]

HERO_OVERLOAD_PKG = "hero_ros2"


@dataclass
class Lvl1Node:
    executable: str
    ms_params: Lvl1Param
    other_params: dict[str, Any]


def arm_prefix(limb_index: int) -> str:
    return f"leg{limb_index}_"


def wheel_prefix(limb_index: int) -> str:
    return f"wheel{limb_index}_"


def read_limb_id() -> str:
    """Reads the LIMB_ID env var

    Replaces empty var with ALL
    """
    tmp_env_id = getenv("LIMB_ID")
    if tmp_env_id is None or tmp_env_id == "":
        env_id = "ALL"
    else:
        env_id = tmp_env_id
    return env_id


#: stores the limb id, once to avoid calling the function multiple times
ENV_LIMB_ID: str = read_limb_id()


def collapse_limb_id(limb_id: Oli = None) -> str:
    """collapses an optional limb id onto a str limb id.

    If limb id is none, the env var is retreived.
    """
    if limb_id is None:
        return ENV_LIMB_ID
    else:
        return limb_id


def parse_id(
    limb_id: Oli = None,
) -> Optional[Tuple[RobotBrands, ModuleType, int, int]]:
    """Parses the limb id into it's individual components.

    id style must be:
        - mh-w-v1-2 : moonbot hero wheel v1 number 2
        - mh-a-v2-1 : moonbot hero arm v2 number 1
        - mg-a-v3-1 : moonbot gustavo arm v3 number 1

    Returns:
        tuple(brand, module, version, number)
        None: if no id is present (id is "ALL") and all node shall launch

    """
    id: str = collapse_limb_id(limb_id)
    if id == "ALL":
        return None

    brand: RobotBrands
    module: ModuleType
    version: int

    split = id.split("-")
    ex = ValueError(f"Could not interpret the LIMB_ID: {id} {split=}")
    if len(split) != 4:
        raise ex

    if split[0] == "hero":
        brand = "hero"
    elif split[0] == "gustavo":
        brand = "gustavo"
    else:
        raise ex
    if split[1] == "arm":
        module = "arm"
    elif split[1] == "wheel":
        module = "wheel"
    else:
        raise ex
    if split[2] == "v1":
        version = 1
    elif split[2] == "v2":
        version = 2
    elif split[2] == "v3":
        version = 3
    else:
        raise ex
    if not (split[3].isdigit()):
        raise ex
    number = int(split[3])

    return (brand, module, version, number)


def this_is_a_real_robot(limb_id: Oli = None) -> bool:
    """Asserts the limb id is valid

    Args:
        limb_id:

    Returns:

    """
    try:
        parsed = parse_id(limb_id)
        chat_this_is_real = parsed is not None
        return chat_this_is_real
    except ValueError:
        return False


class RobotModule(ABC):
    """Abtsract class representing a physical robot module.

    It needs to be completed by detailed robot modules implementations and
    characteristics.

    Attributes:
        brand:
        module:
        version:
        number:
    """

    brand: RobotBrands
    module: ModuleType
    version: int
    number: int
    add_joints: List[str]
    simu_mode: bool
    down_from: int
    up_to: int
    MS_PACKAGE = "motion_stack"

    urdf_assembler: Union[urdf.base.Arm, urdf.base.Wheel]

    def __init__(
        self,
        brand: RobotBrands,
        module: ModuleType,
        version: int,
        number: int,
    ) -> None:
        self.brand = brand
        self.module = module
        self.version = version
        self.number = number
        self.executable: List[str] = [
            "python3",
            "-m",
            "ms_moonbot_zero.ms_moonbot_zero.lvl1",
        ]
        self.add_joints = []
        self._end_effector = None
        self._start_effector = None
        self.urdf_overide: str | None = None
        self.namespace = limb_ns(self.limb_index)
        self.simu_mode = parse_id() == None
        self.down_from = 1
        self.up_to = 2

    def get_parsed_id(self) -> Tuple[RobotBrands, ModuleType, int, int]:
        return (self.brand, self.module, self.version, self.number)

    @property
    def joint_prefix(self):
        return self.urdf_assembler.prefix

    @property
    def start_effector(self) -> str:
        """getter setter, because it depends on the assembler and also needs to
        be changed by hand sometimes"""
        if self._start_effector is None:
            return ""
            # return self.urdf_assembler.root
        return self._start_effector

    @start_effector.setter
    def start_effector(self, value: str):
        self._start_effector = value

    def compile_urdf(self):
        if self.urdf_overide is not None:
            return self.urdf_overide
        else:
            return self.urdf_assembler.compile()

    @property
    def end_effector(self) -> str:
        """getter setter, because it depends on the assembler and also needs to
        be changed by hand sometimes"""
        if self._end_effector is None:
            return self.urdf_assembler.leaf
        return self._end_effector

    @end_effector.setter
    def end_effector(self, value: str):
        self._end_effector = value

    def levels(self) -> Set[int]:
        """levels to launch"""
        lvls: Set[int]
        if self.is_arm:
            lvls = {1, 2}
        else:
            lvls = {1}
        lvls = {l for l in lvls if self.down_from <= l <= self.up_to}
        return lvls

    @property
    @abstractmethod
    def limb_index(self) -> int:
        """
        limb_number parameter of the motion stack.
        This is not the limb_id, this is just an `int`.

        Two modules should never share the same limb_number, this function
        should deduce a unique limb_number from the limb_id
        """
        pass

    @property
    def is_arm(self) -> bool:
        """
        True, if this is a manipulator arm where IK shall be made available.
        """
        return self.module == "arm"

    @property
    def is_active(self) -> bool:
        """
        True if this limb shall be launched (running simulation, or this is
                                                 the PC with corresponding ID)
        """
        interpreted = parse_id()
        if interpreted is None:
            return True
        it_is_me = interpreted == (self.brand, self.module, self.version, self.number)
        return it_is_me

    def get_lvl1(self) -> list[Lvl1Node]:
        exec = self.executable
        if not 1 in self.levels():
            return []
        params = Lvl1Param()
        params.end_effector_name = self.end_effector
        params.start_effector_name = self.start_effector
        params.namespace = self.namespace
        params.urdf = self.compile_urdf()
        params.add_joint = self.add_joints
        additionals = {}
        return [Lvl1Node(executable=exec, ms_params=params, other_params=additionals)]

    def friendly_display(self):
        index_disp = (
            f"{Fore.GREEN}ind-{self.limb_index:3}{Fore.RESET}"
            if self.is_active
            else f"{Fore.RED}skipped{Fore.RESET}"
        )
        simu = " (sim)" if self.simu_mode else ""
        return (
            f"[{index_disp}] {self.brand:8} {self.module:6}"
            f" v{self.version} #{self.number}{simu}"
        )
