import time
from typing import Any

import numpy as np
import quaternion as qt
from motion_stack.utils.pose import Pose
from motion_stack.utils.time import Time


def pose_to_cycl(pose: Pose):
    from ros2_pyterfaces.cyclone import all_msgs as cycl_msgs

    q = qt.as_float_array(pose.quat)
    w, x, y, z = q

    return cycl_msgs.Transform(
        translation=cycl_msgs.Vector3(
            x=float(pose.xyz[0]),
            y=float(pose.xyz[1]),
            z=float(pose.xyz[2]),
        ),
        rotation=cycl_msgs.Quaternion(
            x=float(x),
            y=float(y),
            z=float(z),
            w=float(w),
        ),
    )


def pose_to_cydr(pose: Pose):
    from ros2_pyterfaces.cydr import all_msgs as cydr_msgs

    q = qt.as_float_array(pose.quat)
    # numpy-quaternion order is [w, x, y, z]
    w, x, y, z = q

    return cydr_msgs.Transform(
        translation=cydr_msgs.Vector3(
            x=np.float64(pose.xyz[0]),
            y=np.float64(pose.xyz[1]),
            z=np.float64(pose.xyz[2]),
        ),
        rotation=cydr_msgs.Quaternion(
            x=np.float64(x),
            y=np.float64(y),
            z=np.float64(z),
            w=np.float64(w),
        ),
    )


def cycl_to_pose(msg: Any, stamp: Time | None = None) -> Pose:
    from ros2_pyterfaces.cyclone import all_msgs as cycl_msgs

    msg: cycl_msgs.Transform

    if stamp is None:
        stamp = Time(time.time_ns())

    xyz = np.array(
        [
            float(msg.translation.x),
            float(msg.translation.y),
            float(msg.translation.z),
        ],
        dtype=float,
    )

    quat = np.quaternion(
        float(msg.rotation.w),
        float(msg.rotation.x),
        float(msg.rotation.y),
        float(msg.rotation.z),
    )

    return Pose(
        time=stamp,
        xyz=xyz,
        quat=quat,
    )


def cydr_to_pose(msg: Any, stamp: Time | None = None) -> Pose:
    from ros2_pyterfaces.cydr import all_msgs as cydr_msgs

    msg: cydr_msgs.Transform

    if stamp is None:
        stamp = Time(time.time_ns())

    xyz = np.array(
        [
            float(msg.translation.x),
            float(msg.translation.y),
            float(msg.translation.z),
        ],
        dtype=float,
    )

    quat = np.quaternion(
        float(msg.rotation.w),
        float(msg.rotation.x),
        float(msg.rotation.y),
        float(msg.rotation.z),
    )

    return Pose(
        time=stamp,
        xyz=xyz,
        quat=quat,
    )
