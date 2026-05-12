import copy
import logging
from typing import Optional, TypeAlias, Dict

import numpy as np

from motion_stack.utils.math import qt
from motion_stack.utils.pose import Pose
from motion_stack.utils.time import Time

logger = logging.getLogger(__name__)

LimbNumber: TypeAlias = int
MultiPose: TypeAlias = Dict[LimbNumber, Pose]


def impose_pose(onto: Optional[Pose], fromm: Optional[Pose]) -> Pose:
    """Replaces values of onto with values fromm, unless from is None."""
    if onto is None and fromm is None:
        return Pose(
            time=Time(0),
            xyz=np.full(3, np.nan),
            quat=qt.one,
        )

    if onto is None:
        return fromm.copy()

    if fromm is None:
        return onto.copy()

    time = fromm.time if fromm.time is not None else onto.time
    xyz = fromm.xyz if fromm.xyz is not None else onto.xyz
    quat = fromm.quat if fromm.quat is not None else onto.quat

    return Pose(
        time=time,
        xyz=copy.deepcopy(xyz),
        quat=copy.deepcopy(quat),
    )


class PoseBuffer:
    """Accumulates FK/end-effector Pose feedback.

    - keeps latest pose per limb
    - skips older timestamped data
    - allows partial pose updates if xyz/quat are None
    """

    def __init__(self) -> None:
        self.accumulated: MultiPose = {}

    def push(self, poses: MultiPose) -> MultiPose:
        """Push poses into the buffer.

        Returns:
            Accepted poses, after rejecting old timestamped data.
        """
        if len(poses) == 0:
            return {}

        not_old: MultiPose = {}

        for limb, pose in poses.items():
            if pose.time is None or pose.time == 0:
                not_old[limb] = pose
                continue

            acc = self.accumulated.get(limb)
            if acc is None:
                not_old[limb] = pose
                continue

            if acc.time is None or acc.time == 0:
                not_old[limb] = pose
                continue

            if acc.time <= pose.time:
                not_old[limb] = pose

        if logger.isEnabledFor(logging.DEBUG):
            never_seen = set(not_old.keys()) - set(self.accumulated.keys())
            if never_seen:
                logger.debug("new pose feedback buffered: %s", never_seen)

        for limb, pose in not_old.items():
            self.accumulated[limb] = impose_pose(
                self.accumulated.get(limb),
                pose,
            )

        return not_old

    def clear(self) -> None:
        self.accumulated.clear()

    def copy(self) -> MultiPose:
        return copy.deepcopy(self.accumulated)
