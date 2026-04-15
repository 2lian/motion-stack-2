import time
from tempfile import NamedTemporaryFile
from typing import List


from ..modules.base import RobotModule
from ..urdf.base import Joint, Link, wrap_in_robot


def link_urdf_nodes(modules: List[RobotModule], urdf: str) -> list:
    f = NamedTemporaryFile(mode="w", suffix=".urdf", prefix="ms_lvl1_", delete=False)
    f.write(urdf)
    f.close()

    for m in modules:
        m.urdf_overide = f.name

    return modules
