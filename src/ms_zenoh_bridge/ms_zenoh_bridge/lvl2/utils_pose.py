import time

import msgspec
import numpy as np
import quaternion as qt
from motion_stack.utils.pose import Pose
from motion_stack.utils.time import Time


class WirePose(msgspec.Struct, array_like=True):
    sec: int
    nano: int
    xyz: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]


_ENCODER = msgspec.msgpack.Encoder()
_DECODER = msgspec.msgpack.Decoder(WirePose)


def pose_to_wire(pose: Pose, stamp: Time | None = None) -> bytes:
    if stamp is None:
        stamp = pose.time if pose.time is not None else Time(time.time_ns())

    sec, nano = stamp.to_parts()

    q = qt.as_float_array(pose.quat)
    w, x, y, z = q

    msg = WirePose(
        sec=int(sec),
        nano=int(nano),
        xyz=(
            float(pose.xyz[0]),
            float(pose.xyz[1]),
            float(pose.xyz[2]),
        ),
        quat_wxyz=(
            float(w),
            float(x),
            float(y),
            float(z),
        ),
    )

    return _ENCODER.encode(msg)


def wire_to_pose(payload: bytes) -> Pose:
    msg = _DECODER.decode(payload)
    timestamp = Time.from_parts(msg.sec, msg.nano)

    xyz = np.array(msg.xyz, dtype=float)
    w, x, y, z = msg.quat_wxyz

    quat = np.quaternion(
        float(w),
        float(x),
        float(y),
        float(z),
    )

    return Pose(
        time=timestamp,
        xyz=xyz,
        quat=quat,
    )
