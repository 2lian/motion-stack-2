"""
Python API to sync the movement of several end-effectors.

This is the transport-free / Pythonic lvl2 IK syncer.
ROS, Zenoh, or other runtimes should connect through BaseSub streams instead
of subclassing runtime-specific Future implementations.
"""

import asyncio
import copy
import time
import warnings
from abc import ABC, abstractmethod
from typing import (
    Any,
    Awaitable,
    Callable,
    Coroutine,
    Dict,
    Optional,
    Set,
    TypeAlias,
    Union,
)

import asyncio_for_robotics as afor
import numpy as np
from asyncio_for_robotics import BaseSub

from motion_stack.utils.hypersphere_clamp import clamp_multi_xyz_quat, fuse_xyz_quat
from motion_stack.utils.math import Flo3, Quaternion, qt, qt_normalize
from motion_stack.utils.pose import Pose, VelPose, XyzQuat
from motion_stack.utils.pose_state import LimbNumber, MultiPose, PoseBuffer
from motion_stack.utils.time import Time


class SensorSyncWarning(Warning):
    pass


def make_delta_time():
    last = time.monotonic()

    def elapsed() -> float:
        nonlocal last
        now = time.monotonic()
        dt = now - last
        last = now
        return dt

    return elapsed


class IkSyncer(ABC):
    r"""Synchronizes several end-effector trajectories safely.

    One instance can execute one active target trajectory at a time.

    The trajectory interpolates between:
        - previous sent pose, or current sensor pose if no previous pose exists
        - target pose

    Strategies:
        - lerp
        - unsafe
        - speed_safe

    Args:
        interpolation_delta:
            Allowed Cartesian/orientation divergence before slowing down.
            xyz is usually mm, quat is rad.
        on_target_delta:
            Error threshold for considering the motion finished.
    """

    _COMMAND_DONE_DELTA: XyzQuat[float, float] = XyzQuat(
        0.01,
        np.deg2rad(0.01),
    )

    def __init__(
        self,
        interpolation_delta: XyzQuat[float, float] = XyzQuat(40, np.deg2rad(4)),
        on_target_delta: XyzQuat[float, float] = XyzQuat(40, np.deg2rad(4)),
    ) -> None:
        self._interpolation_delta: XyzQuat[float, float] = interpolation_delta
        self._on_target_delta: XyzQuat[float, float] = on_target_delta

        self.last_future: asyncio.Future = asyncio.Future()

        # If True, keep sending commands until the sensor reaches the target.
        # Better for lossy networks, more traffic.
        self.SEND_UNTIL_DONE: bool = True

        self.__previous: MultiPose = {}
        self._last_valid: MultiPose = {}

        self._trajectory_task: Callable[[], None] = lambda *_: None

    def execute(self):
        """Executes one step of the active trajectory.

        This must be called frequently, or automatically by AsyncIkSyncer.
        """
        self._trajectory_task()

    def clear(self):
        """Reset trajectory memory so the next motion starts from sensor pose."""
        self.__previous = {}
        self._last_valid = {}

    def lerp(self, target: MultiPose) -> Awaitable:
        """Start a synchronized LERP trajectory toward target."""
        return self._make_motion(target, self.lerp_toward)

    def unsafe(self, target: MultiPose) -> Awaitable:
        """Send final target directly.

        Dangerous if lower layers or network are unreliable.
        """
        return self._make_motion(target, self.unsafe_toward)

    def asap(self, target: MultiPose) -> Awaitable:
        """Placeholder for ASAP Cartesian motion.

        Currently not implemented, same as old version.
        """
        return self._make_motion(target, self.asap_toward)

    def speed_safe(
        self,
        target: dict[LimbNumber, VelPose],
        delta_time: Union[float, Callable[[], float]],
    ) -> Awaitable:
        """Cartesian velocity control.

        VelPose.lin is linear velocity, usually mm/s.
        VelPose.rvec is rotational velocity as rotation vector, rad/s.
        """
        if not callable(delta_time):
            delta_time = lambda x=delta_time: x

        target_mp: MultiPose = {
            limb: Pose(
                Time(0),
                target[limb].lin,
                qt.from_rotation_vector(target[limb].rvec),
            )
            for limb in target.keys()
        }

        def _step_speed(
            _target_mp: MultiPose,
            _start: Optional[MultiPose] = None,
        ) -> bool:
            dt = delta_time()
            rel_offsets: MultiPose = {}

            for limb, vp in target.items():
                delta_xyz = vp.lin * dt
                delta_quat = qt.from_rotation_vector(vp.rvec * dt)

                rel_offsets[limb] = Pose(
                    Time(0),
                    delta_xyz,
                    qt_normalize(delta_quat),
                )

            abs_targets = self.abs_from_rel(rel_offsets)
            self.lerp_toward(abs_targets)
            return False

        return self._make_motion(target_mp, _step_speed)

    def abs_from_rel(self, offset: MultiPose) -> MultiPose:
        """Convert relative end-effector offsets into absolute poses."""
        track = set(offset.keys())
        prev = self._previous_point(track)

        return {
            key: Pose(
                offset[key].time,
                prev[key].xyz + qt.rotate_vectors(prev[key].quat, offset[key].xyz),
                prev[key].quat * offset[key].quat,
            )
            for key in track
        }

    def _send_to_lvl2(self, ee_targets: MultiPose):
        self.send_to_lvl2(ee_targets)

    @abstractmethod
    def send_to_lvl2(self, ee_targets: MultiPose):
        """Send IK targets to lvl2.

        Runtime/interface layer implements this.
        """
        ...

    @property
    def _sensor(self) -> MultiPose:
        return self.sensor

    @property
    @abstractmethod
    def sensor(self) -> MultiPose:
        """Current FK/end-effector sensor pose by limb."""
        ...

    def ready(self, limbs: Union[Set[LimbNumber], MultiPose]) -> tuple[bool, set[int]]:
        if isinstance(limbs, dict):
            limbs = set(limbs.keys())

        available = set(self._sensor.keys())
        missing = limbs - available
        return len(missing) == 0, missing

    def _previous_point(self, track: Set[LimbNumber]) -> MultiPose:
        missing = track - set(self.__previous.keys())

        if not missing:
            return self.__previous

        sensor = self._sensor
        available = set(sensor.keys())

        for limb in missing & available:
            self.__previous[limb] = sensor[limb]

        return self.__previous

    def _update_previous_point(self, data: MultiPose) -> None:
        self.__previous.update(copy.deepcopy(data))

    def _get_last_valid(self, track: Set[LimbNumber]) -> MultiPose:
        missing = track - set(self._last_valid.keys())

        if not missing:
            return self._last_valid

        sensor = self._sensor
        available = set(sensor.keys())

        for limb in missing & available:
            self._last_valid[limb] = sensor[limb]

        return self._last_valid

    def _set_last_valid(self, data: MultiPose) -> None:
        self._last_valid.update(copy.deepcopy(data))

    def _center_and_previous(
        self, track: Set[LimbNumber]
    ) -> tuple[MultiPose, MultiPose]:
        center = self._sensor

        assert set(center.keys()) >= track, (
            "Sensor does not have required end-effector data. "
            f"Target limbs {track} are not a subset of sensor set {set(center.keys())}."
        )

        previous = self._previous_point(track)

        assert set(previous.keys()) >= track, (
            "Previous step does not have required end-effector data. "
            f"Target limbs {track} are not a subset of previous set {set(previous.keys())}."
        )

        return center, previous

    def _get_lerp_step(
        self,
        start: MultiPose,
        target: MultiPose,
    ) -> tuple[MultiPose, bool]:
        track = set(target.keys())
        order = list(target.keys())

        center, _ = self._center_and_previous(track)
        previous = start

        clamped = clamp_multi_xyz_quat(
            start=_order_dict2list(order, previous),
            center=_order_dict2list(order, center),
            end=_order_dict2list(order, target),
            radii=self._interpolation_delta,
        )

        validity = not bool(np.any(np.isnan(fuse_xyz_quat(clamped))))

        out_dict: MultiPose = {
            limb: Pose(Time(0), xyzquat.xyz, xyzquat.quat)
            for limb, xyzquat in zip(order, clamped)
        }

        return out_dict, validity

    def _get_asap_step(
        self,
        start: MultiPose,
        target: MultiPose,
    ) -> tuple[MultiPose, bool]:
        raise NotImplementedError("IkSyncer.asap is not implemented yet.")

    def _get_unsafe_step(
        self,
        start: MultiPose,
        target: MultiPose,
    ) -> tuple[MultiPose, bool]:
        return target, True

    def _traj_toward(
        self,
        target: MultiPose,
        step_func: Callable[[MultiPose, MultiPose], tuple[MultiPose, bool]],
        start: Optional[MultiPose] = None,
    ) -> bool:
        order = list(target.keys())
        track = set(order)

        if len(target) == 0:
            return True

        prev = self._previous_point(track)

        if start is None:
            start = prev

        is_on_final = _multipose_close(
            track,
            target,
            prev,
            atol=self._COMMAND_DONE_DELTA,
        )

        if not is_on_final:
            next_pose, valid = step_func(start, target)

            if valid:
                self._set_last_valid(next_pose)
            else:
                sens, _ = self._center_and_previous(track)
                next_pose, valid = step_func(
                    sens,
                    self._get_last_valid(track),
                )
        else:
            next_pose = target

        if self.SEND_UNTIL_DONE:
            self._send_to_lvl2(next_pose)
            self._update_previous_point(next_pose)

        if not is_on_final:
            return False

        on_target = _multipose_close(
            track,
            target,
            self._sensor,
            atol=self._on_target_delta,
        )

        return on_target

    def unsafe_toward(
        self,
        target: MultiPose,
        start: Optional[MultiPose] = None,
    ) -> bool:
        return self._traj_toward(target, self._get_unsafe_step, start=start)

    def asap_toward(
        self,
        target: MultiPose,
        start: Optional[MultiPose] = None,
    ) -> bool:
        return self._traj_toward(target, self._get_asap_step, start=start)

    def lerp_toward(
        self,
        target: MultiPose,
        start: Optional[MultiPose] = None,
    ) -> bool:
        return self._traj_toward(target, self._get_lerp_step, start=start)

    def _define_pose(self, multi_pose: MultiPose):
        """Replace None fields with previous pose data in-place."""
        prev = self._previous_point(set(multi_pose.keys()))

        for limb, pose in multi_pose.items():
            if pose.time is None:
                pose.time = Time(0)

            if pose.xyz is None:
                pose.xyz = prev[limb].xyz

            if pose.quat is None:
                pose.quat = prev[limb].quat

    def _make_motion(
        self,
        target: MultiPose,
        toward_func: Callable[[MultiPose, Optional[MultiPose]], bool],
    ) -> Awaitable:
        future = asyncio.Future()

        self.last_future.cancel()
        self.last_future = future

        if len(target) == 0:
            future.set_result(True)
            return future

        track = set(target.keys())

        sens, prev = self._center_and_previous(track)

        has_moved_since_last_time = _multipose_close(
            track,
            prev,
            sens,
            atol=self._interpolation_delta,
        )

        if not has_moved_since_last_time:
            warnings.warn(
                "Syncer is out of sync with sensor data. "
                "Something else may have moved the end-effectors. "
                "`syncer.clear()` will be called automatically, so the trajectory "
                "will restart from the current sensor pose.",
                SensorSyncWarning,
                stacklevel=3,
            )
            self.clear()
            sens, prev = self._center_and_previous(track)

        self._define_pose(target)

        target = copy.deepcopy(target)
        prev = copy.deepcopy(prev)

        stop = [False]

        def step_toward_target(future=future, stop=stop):
            if stop[0]:
                return

            if future.cancelled():
                stop[0] = True
                return

            if future.done():
                stop[0] = True
                return

            move_done = toward_func(target, prev)

            if move_done:
                future.set_result(move_done)

        self._trajectory_task = step_toward_target

        return future

    def __del__(self):
        self.last_future.cancel()


class AsyncIkSyncer(IkSyncer):
    """Async BaseSub implementation of IkSyncer.

    lvl2_output.tip_pos  -> fk_feedback_input
    ik_target_output     -> lvl2_input.set_ik

    """

    def __init__(
        self,
        interpolation_delta: XyzQuat[float, float] = XyzQuat(40, np.deg2rad(4)),
        on_target_delta: XyzQuat[float, float] = XyzQuat(40, np.deg2rad(4)),
        batch_time: float = 1 / 100,
    ) -> None:
        super().__init__(interpolation_delta, on_target_delta)
        self.sensor_input: BaseSub[MultiPose] = BaseSub()
        self.command_output: BaseSub[MultiPose] = BaseSub()
        self._feedback_buffer = PoseBuffer()
        self._iterator = self.sensor_input.listen_reliable(queue_size=1000)
        self._execute_event = asyncio.Event()
        self._sensor_update_cond = asyncio.Condition()
        self.batch_time: float = batch_time

    async def run(self) -> None:
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self._consume_task(), name="async_ik_syncer_consume")
            tg.create_task(self._execute_task(), name="async_ik_syncer_execute")

    def _make_motion(
        self,
        target: MultiPose,
        toward_func: Callable[[MultiPose, Optional[MultiPose]], bool],
    ) -> Awaitable:
        result = super()._make_motion(target, toward_func)
        self.execute()
        return result

    async def wait_ready(self, limbs: Union[Set[LimbNumber], MultiPose]) -> None:
        """Wait until FK feedback is available for requested limbs."""
        async with self._sensor_update_cond:
            await self._sensor_update_cond.wait_for(lambda: self.ready(limbs)[0])

    def send_to_lvl2(self, ee_targets: MultiPose) -> None:
        """Emit IK targets for lvl2 input.set_ik."""
        self.command_output._input_data_asyncio(copy.deepcopy(ee_targets))

    @property
    def sensor(self) -> MultiPose:
        return self._feedback_buffer.accumulated

    async def _consume_task(self) -> None:
        async for multipose in self._iterator:
            accepted = self._feedback_buffer.push(copy.deepcopy(multipose))

            if len(accepted) == 0:
                continue

            async with self._sensor_update_cond:
                self._sensor_update_cond.notify_all()

        self._execute_event.set()

    @afor.scoped
    async def _execute_task(self) -> None:
        while True:
            await self._execute_event.wait()
            await asyncio.sleep(self.batch_time)
            self.execute()
            self._execute_event.clear()
