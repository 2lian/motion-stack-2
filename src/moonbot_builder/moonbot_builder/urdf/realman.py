from .base import Arm, UrdfModule


class RealmanModule(UrdfModule):
    ros_package = "realman_interface"

    def __init__(self, limb_number: int):
        super().__init__(limb_number)
        self.name += "rm"


class RealmanArm(Arm, RealmanModule):
    package_relative_mesh_path = "meshes/"
    jinja_file = "realman_75.jinja.urdf"
    mesh_extension = "STL"

    def __init__(self, limb_number: int):
        super().__init__(limb_number)
        self.name += f"rm-{limb_number}"
