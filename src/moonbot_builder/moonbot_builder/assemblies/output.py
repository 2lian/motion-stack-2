"""
This output the urdf string from an assembly in the terminal. That's it.
Run this using `python3 -m moonbot_launcher.assemblies.output` (workspace needs to be built and sourced).
Change this file to output something else.

The robot modules you wanna use need to be sourced before running, so hero_ros2 or moonbot_g package need to be sourced.
"""

from moonbot_launcher.assemblies import alone

from ..modules.gusta import MgArmV3
from ..modules.hero import MhArmV1, MhArmV2, MhArmV3, MhWheelV1, MhWheelV2, MhCargo, MhCargoV2
from ..urdf.base import wrap_in_robot
from . import dragon, minimal, vehicule, cargo, cargo_v2

if __name__ == "__main__":
    # urdf = dragon.raw_urdf(
    #     manipulator=MhArmV2(1),
    #     front_wheel=MhWheelV2(1),
    #     bridge=MhArmV2(2),
    #     back_wheel=MhWheelV2(2),
    # )

    # urdf = vehicule.raw_urdf(
    #     front_wheel=MhWheelV2(1),
    #     bridge=MhArmV2(1),
    #     back_wheel=MhWheelV2(2),
    # )

    # urdf = alone.raw_urdf(
    #     modules=[
    #         # MhArmV2(2),
    #         MgArmV3(2),
    #         # MgArmV3(3),
    #         # MhArmV1(1),
    #         # MgArmV3(5),
    #     ],
    #     base_link_name="base_link_arm",
    # )

    # Moonbot Cargo Mode: 2 wheels + 2 arms + central box
    # CargoV1: V1 wheels + V1 arms
    # urdf = cargo.raw_urdf(
    #     left_wheel=MhWheelV1(1),
    #     right_wheel=MhWheelV1(2), 
    #     left_arm=MhArmV1(3),
    #     right_arm=MhArmV1(4),
    #     cargo_box=MhCargo(1),
    # )

    # CargoV2: V2 wheels + V3 arms
    # urdf = cargo.raw_urdf(
    #     left_wheel=MhWheelV2(1),
    #     right_wheel=MhWheelV2(2),
    #     left_arm=MhArmV3(3),
    #     right_arm=MhArmV3(4),
    #     cargo_box=MhCargo(1),
    # )

    # CargoV2 with Sled: V2 wheels + V3 arms + sled body
    urdf = cargo_v2.raw_urdf(
        left_wheel=MhWheelV2(1),
        right_wheel=MhWheelV2(2),
        left_arm=MhArmV3(3),
        right_arm=MhArmV3(4),
        cargo_box=MhCargoV2(1),  # Uses sled instead of cargo_body
    )

    # urdf = minimal.raw_urdf(
        # arm=MhArmV2(1),
        # wheel=MhWheelV2(1),
    # )
    print(wrap_in_robot(urdf))
