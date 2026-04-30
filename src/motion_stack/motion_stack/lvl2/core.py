import asyncio
import dataclasses
import logging
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Final, Optional, Self, TypeAlias

import asyncio_for_robotics as afor
import dacite
import matplotlib
import numpy as np
import quaternion as qt
import roboticstoolbox as rtb
import ujson
from asyncio_for_robotics.core.sub import BaseSub
from colorama import Fore, Style
from numpy.typing import NDArray
from roboticstoolbox import ETS, Link, Robot
from roboticstoolbox.robot.ET import SE3

matplotlib.use("Agg")

from motion_stack.lvl1.core import JStateBatch
from motion_stack.lvl1.joint_api import AsyncJointSyncer
from motion_stack.utils.joint_state import JState, subdict
from motion_stack.utils.pose import Pose
from motion_stack.utils.printing import list_cyanize
from motion_stack.utils.robot_parsing import load_set_urdf_raw, make_ee
from motion_stack.utils.time import Time

from ..rtb_fix.patch import patch

patch()

if TYPE_CHECKING:
    from motion_stack.lvl1.core import Lvl1Param

logger = logging.getLogger(__name__)

float_formatter = "{:.2f}".format
np.set_printoptions(formatter={"float_kind": float_formatter})

PoseBatch: TypeAlias = dict[int, Pose]


@dataclass
class Lvl2Param:
    urdf: str = ""
    namespace: str = "ms"
    end_effector_name: None | str | int = "ALL"
    start_effector_name: None | str = None
    mvmt_update_rate: float = 100.0
    ignore_limits: bool = False
    angle_syncer_delta: float = np.deg2rad(5.0)

    joint_buffer: JState = dataclasses.field(
        default_factory=lambda: JState(
            name="",
            time=Time.sn(sec=2),
            position=np.deg2rad(0.05),
            velocity=np.deg2rad(0.01),
            effort=np.deg2rad(0.001),
        )
    )

    batch_time: float = 0.001
    fk_publish_hz: float = 100.0
    monitor_timeout: float = 3.0
    reset_last_sent_sec: float = 0.5
    target_xyz_is_mm: bool = True
    fk_xyz_output_mm: bool = True

    def to_json(self) -> str:
        return ujson.dumps(dataclasses.asdict(self), indent=2)

    @classmethod
    def from_json(cls, json_str: str) -> Self:
        data = ujson.loads(json_str)

        urdf = data.get("urdf", "")
        try:
            if urdf and Path(urdf).is_file():
                data["urdf"] = Path(urdf).read_text()
        except OSError:
            pass

        return dacite.from_dict(cls, data, dacite.Config())

    @classmethod
    def from_lvl1_param(
        cls,
        lvl1: "Lvl1Param",
        *,
        end_effector_name: None | str | int | object = dataclasses.MISSING,
        start_effector_name: None | str | object = dataclasses.MISSING,
        angle_syncer_delta: float | None = None,
        fk_publish_hz: float | None = None,
        monitor_timeout: float | None = None,
        reset_last_sent_sec: float | None = None,
        target_xyz_is_mm: bool | None = None,
        fk_xyz_output_mm: bool | None = None,
    ) -> Self:
        data = {
            "urdf": lvl1.urdf,
            "namespace": lvl1.namespace,
            "end_effector_name": lvl1.end_effector_name,
            "start_effector_name": lvl1.start_effector_name,
            "mvmt_update_rate": lvl1.mvmt_update_rate,
            "ignore_limits": lvl1.ignore_limits,
            "joint_buffer": lvl1.joint_buffer,
            "batch_time": lvl1.batch_time,
        }

        if end_effector_name is not dataclasses.MISSING:
            data["end_effector_name"] = end_effector_name

        if start_effector_name is not dataclasses.MISSING:
            data["start_effector_name"] = start_effector_name

        if angle_syncer_delta is not None:
            data["angle_syncer_delta"] = angle_syncer_delta

        if fk_publish_hz is not None:
            data["fk_publish_hz"] = fk_publish_hz

        if monitor_timeout is not None:
            data["monitor_timeout"] = monitor_timeout

        if reset_last_sent_sec is not None:
            data["reset_last_sent_sec"] = reset_last_sent_sec

        if target_xyz_is_mm is not None:
            data["target_xyz_is_mm"] = target_xyz_is_mm

        if fk_xyz_output_mm is not None:
            data["fk_xyz_output_mm"] = fk_xyz_output_mm

        return cls(**data)


lvl2_default: Final[Lvl2Param] = Lvl2Param()


class IKCore:
    def __init__(self, params: Lvl2Param = lvl2_default) -> None:
        self.PARAMS: Lvl2Param = params

        # Input from lvl1 joint read.
        self.joint_state_input: BaseSub[JStateBatch] = BaseSub()

        # Input from lvl3 / high-level Cartesian target.
        self.ik_target_input: BaseSub[Pose] = BaseSub()

        # Output to lvl3 / high-level FK pose stream.
        self.fk_output: BaseSub[Pose] = BaseSub()

        # Internal async joint syncer used to safely send lvl1 joint commands.
        self.joint_syncer = AsyncJointSyncer(
            interpolation_delta=self.PARAMS.angle_syncer_delta,
            on_target_delta=self.PARAMS.angle_syncer_delta,
            buffer=self.PARAMS.joint_buffer,
            batch_time=self.PARAMS.batch_time,
        )

        # Output to lvl1 motor/joint command.
        self.joint_command_output: BaseSub[JStateBatch] = (
            self.joint_syncer.command_output
        )

        (
            self.model,
            self.et_chain,
            self.joint_names,
            self.joints_objects,
            self.last_link,
        ) = self.setup_urdf()

        self.end_link: Link = self.last_link  # type: ignore
        self.sub_model: Robot = rtb.Robot(self.et_chain)

        self.angles: NDArray = np.empty(len(self.joint_names), dtype=float)
        self.angles[:] = np.nan

        self.last_sent: NDArray = self.angles.copy()
        self.last_time_ik: Time = Time(0)

        self.ready: bool = False

        self._joint_state_iter = self.joint_state_input.listen_reliable(queue_size=1000)
        self._ik_target_iter = self.ik_target_input.listen_reliable(queue_size=1000)
        self._joint_command_iter = self.joint_command_output.listen_reliable(
            queue_size=1000
        )

        print(
            f"Base_link: {Fore.CYAN}{self.model.base_link.name}{Fore.RESET}\n"
            f"Chain: {self.joint_names}\n"
            f"End effector: {Fore.CYAN}{self.end_link.name}{Fore.RESET}"
        )

    async def run(self):
        async with asyncio.TaskGroup() as tg:
            tg.create_task(self.joint_syncer.run(), name="ik_joint_syncer")
            tg.create_task(self._joint_state_loop(), name="ik_joint_state_loop")
            tg.create_task(self._target_loop(), name="ik_target_loop")
            tg.create_task(
                self._command_output_tracking_loop(),
                name="ik_command_output_tracking_loop",
            )
            tg.create_task(self._fk_loop(), name="ik_fk_loop")
            tg.create_task(self.monitor_fk(), name="ik_monitor_fk")

    def setup_urdf(self):
        if self.PARAMS.urdf == "":
            raise ValueError("IKCore requires a URDF. Empty PARAMS.urdf is invalid.")

        end_effector_name = make_ee(self.PARAMS.end_effector_name)

        (
            model,
            et_chain,
            joint_names,
            joints_objects,
            last_link,
        ) = load_set_urdf_raw(
            self.PARAMS.urdf,
            end_effector_name,
            self.PARAMS.start_effector_name,
        )

        if last_link is None:
            raise ValueError(
                f"Could not resolve IK end effector: {self.PARAMS.end_effector_name}"
            )

        return model, et_chain, joint_names, joints_objects, last_link

    async def _joint_state_loop(self):
        async for jsb in self._joint_state_iter:
            self.state_from_lvl1(jsb)

    async def _target_loop(self):
        async for pose in self._ik_target_iter:
            self.ik_target(pose)

    async def _command_output_tracking_loop(self):
        """
        Keeps self.last_sent synchronized with the actual commands emitted by
        AsyncJointSyncer.
        """
        async for jsb in self._joint_command_iter:
            self.save_as_last(jsb)

    @afor.scoped
    async def _fk_loop(self):
        async for _ in afor.Rate(frequency=self.PARAMS.fk_publish_hz).listen():
            if not self.has_full_joint_state:
                continue
            self.send_current_fk()

    @property
    def accumulated_joint_state(self) -> dict[str, JState]:
        return self.joint_syncer.sensor

    @property
    def has_full_joint_state(self) -> bool:
        return not bool(np.any(np.isnan(self.angles)))

    def state_from_lvl1(self, states: JStateBatch):
        joint_of_interest = set(self.joint_names) & set(states.keys())
        jsb = subdict(states, joint_of_interest)

        if len(jsb) == 0:
            return

        self.joint_syncer.sensor_input._input_data_asyncio(jsb)

        for name, js in jsb.items():
            if js.position is None:
                continue

            ind = self.joint_names.index(name)
            self.angles[ind] = js.position

        if self.has_full_joint_state:
            fk = self.send_current_fk()
            self.joint_syncer.execute()

            if not self.ready:
                print(
                    f"Forward Kinematics: "
                    f"{Style.BRIGHT}{Fore.GREEN}FULLY Ready :){Fore.RESET}"
                    f"{Style.RESET_ALL}:\n{fk}"
                )
                self.ready = True

    def ik_target(self, pose: Pose) -> None:
        pose = self._replace_nan(pose)

        if self.PARAMS.target_xyz_is_mm:
            pose = Pose(
                time=pose.time,
                xyz=pose.xyz / 1000.0,
                quat=pose.quat,
            )

        angles = self.find_next_ik(pose)
        self._send_command(angles)

    def compute_raw_ik(
        self,
        pose: Pose,
        start: NDArray,
        compute_budget: Optional[Time] = None,
        mvt_duration: Optional[Time] = None,
    ) -> tuple[Optional[NDArray], bool]:
        if mvt_duration is None:
            delta_time = Time(sec=1 / self.PARAMS.mvmt_update_rate)
        else:
            delta_time = mvt_duration

        if compute_budget is None:
            compute_budget = Time(sec=1 / self.PARAMS.mvmt_update_rate)

        finish_by = self.now() + compute_budget

        motion: SE3 = SE3(pose.xyz)
        motion.A[:3, :3] = qt.as_rotation_matrix(pose.quat)

        if self.angles.shape[0] > 5:
            mask = np.array([1, 1, 1, 1, 1, 1], dtype=float)
        else:
            mask = np.array([1, 1, 1, 0, 0, 0], dtype=float)

        angles = self.angles.copy()
        np.nan_to_num(x=angles, nan=0.0, copy=False)
        np.nan_to_num(x=start, nan=0.0, copy=False)

        trial = -1
        trial_limit = 20

        best_solution: Optional[NDArray] = None
        vel_maybe: float = 1_000_000.0
        valid_solution_found = False

        compute_budget_exceeded = lambda: self.now() > finish_by

        while trial < trial_limit and not compute_budget_exceeded():
            trial += 1
            starting_pose = start.copy()

            if trial <= 0:
                ilimit = 10
                slimit = 1
                tol = 1e-7
            elif trial == 1:
                ilimit = 50
                slimit = 1
                tol = 1e-6
            else:
                ilimit = 50
                slimit = 5
                tol = 1e-5

                stpose = np.empty((slimit, starting_pose.shape[0]), float)
                stpose[:, :] = starting_pose.reshape(1, -1)

                r = np.random.rand(stpose.shape[0], stpose.shape[1]) * 2 - 1
                maxi = 1 / 10
                mini = maxi / 100
                r = r * np.linspace(mini, maxi, slimit, endpoint=True).reshape(-1, 1)

                starting_pose = stpose + r

            ik_result = self.sub_model.ik_LM(
                Tep=motion,
                q0=starting_pose,
                mask=mask,
                ilimit=ilimit,
                slimit=slimit,
                joint_limits=not self.PARAMS.ignore_limits,
                tol=tol,
            )

            sol_found = ik_result[1]
            sol = np.array(ik_result[0], dtype=float)

            sol_im = np.exp(1j * sol)
            start_im = np.exp(1j * start)
            real_delta = np.angle(sol_im / start_im)
            real_angles = start + real_delta

            for ind, _ in enumerate(real_angles):
                limit = self.joints_objects[ind].limit
                if limit is None:
                    continue

                upper = limit.upper
                lower = limit.lower

                if upper is not None and not self.PARAMS.ignore_limits:
                    if real_angles[ind] > upper:
                        real_angles[ind] = sol[ind]

                if lower is not None and not self.PARAMS.ignore_limits:
                    if real_angles[ind] < lower:
                        real_angles[ind] = sol[ind]

            delta = real_angles - start
            dist = float(np.linalg.norm(delta, ord=np.inf))

            _velocity = dist / delta_time.sec()
            del _velocity

            if sol_found:
                if (
                    abs(dist) < abs(self.PARAMS.angle_syncer_delta)
                    or self.PARAMS.angle_syncer_delta <= 0
                ):
                    valid_solution_found = True
                    best_solution = real_angles
                    break

                if dist < vel_maybe:
                    best_solution = real_angles
                    vel_maybe = dist

        if compute_budget_exceeded():
            logger.warning("IK slow, compute terminated")

        return best_solution, valid_solution_found

    def find_next_ik(
        self,
        pose: Pose,
        compute_budget: Optional[Time] = None,
        mvt_duration: Optional[Time] = None,
    ) -> NDArray:
        if not self.ready:
            logger.warning("No angle data available, assuming joints at 0.")

        arrive_time = self.now()
        delta_time = arrive_time - self.last_time_ik
        self.last_time_ik = arrive_time

        ik_is_recent = delta_time < Time(sec=self.PARAMS.reset_last_sent_sec)

        if ik_is_recent:
            start = self.last_sent.copy()
        else:
            start = self.angles.copy()

        if np.any(np.isnan(start)):
            start = self.angles.copy()

        np.nan_to_num(x=start, nan=0.0, copy=False)

        assert start.shape == self.angles.shape

        best_solution, valid_solution_found = self.compute_raw_ik(
            pose,
            start,
            compute_budget=compute_budget,
            mvt_duration=mvt_duration,
        )

        if best_solution is None:
            logger.warning("no IK found :C")
            return start

        if not valid_solution_found:
            logger.warning("no continuous IK found :C")

        return best_solution

    def _send_command(self, angles: NDArray):
        assert self.last_sent.shape == angles.shape
        assert angles.dtype in [float, np.float32, np.float64]

        target = {name: float(angle) for name, angle in zip(self.joint_names, angles)}

        try:
            if self.PARAMS.angle_syncer_delta == 0:
                self.joint_syncer.unsafe(target)
            else:
                self.joint_syncer.lerp(target)
        except AssertionError:
            logger.warning("Joint syncer not ready.")
            return

        self.joint_syncer.execute()

    def save_as_last(self, jsb: JStateBatch):
        posdict = {
            name: js.position for name, js in jsb.items() if js.position is not None
        }

        if len(posdict) == 0:
            return

        for i, name in enumerate(self.joint_names):
            pos = posdict.get(name)
            if pos is None:
                continue
            self.last_sent[i] = pos

    def send_current_fk(self) -> Pose:
        fk = self.current_fk()
        self.fk_output._input_data_asyncio(fk)
        return fk

    def current_fk(self) -> Pose:
        fw_result: list[SE3] = self.sub_model.fkine(self.angles)  # type: ignore

        rot_matrix = np.array(fw_result[-1].R, dtype=float)
        tip_coord: NDArray = np.array(fw_result[-1].t, dtype=float)

        if self.PARAMS.fk_xyz_output_mm:
            tip_coord = tip_coord * 1000.0

        tip_quat: qt.quaternion = qt.from_rotation_matrix(rot_matrix)

        return Pose(
            time=self.now(),
            xyz=tip_coord,
            quat=tip_quat,
        )

    def _replace_nan(self, pose: Pose) -> Pose:
        xyz = pose.xyz
        quat = pose.quat

        fk = None

        if np.any(np.isnan(xyz)):
            if fk is None:
                fk = self.current_fk()

            xyz = fk.xyz.copy()

            if self.PARAMS.fk_xyz_output_mm and self.PARAMS.target_xyz_is_mm:
                pass
            elif self.PARAMS.fk_xyz_output_mm and not self.PARAMS.target_xyz_is_mm:
                xyz = xyz / 1000.0
            elif not self.PARAMS.fk_xyz_output_mm and self.PARAMS.target_xyz_is_mm:
                xyz = xyz * 1000.0

        if np.any(np.isnan(qt.as_float_array(quat))):
            if fk is None:
                fk = self.current_fk()
            quat = fk.quat

        return Pose(time=pose.time, xyz=xyz, quat=quat)

    async def monitor_fk(self):
        active: set[str] = set()
        incoming: set[str] = set()
        needed = set(self.joint_names)

        async def mon():
            nonlocal incoming
            async for js_batch in self.joint_state_input.listen_reliable():
                incoming |= {
                    name
                    for name, js in js_batch.items()
                    if name in needed and js.position is not None
                }

        async def good_display():
            nonlocal active, incoming

            async for _ in afor.Rate(10).listen():
                new = incoming - active

                if new:
                    print(
                        f"Forward Kinematics: "
                        f"{Fore.BLUE}Ready{Fore.RESET} for {list_cyanize(new)}"
                    )

                active |= incoming
                incoming = set()

                if needed - active == set():
                    print(
                        f"Forward Kinematics: "
                        f"{Style.BRIGHT}{Fore.GREEN}FULLY READY :){Fore.RESET}"
                        f"{Style.RESET_ALL}"
                    )
                    return

        async def bad_display():
            await asyncio.sleep(self.PARAMS.monitor_timeout)

            missing = needed - active
            if not missing:
                return

            if missing == needed:
                print(
                    f"Forward Kinematics: "
                    f"{Fore.RED}MISSING ALL :({Fore.RESET} {list_cyanize(missing)}"
                )
            else:
                print(
                    f"Forward Kinematics: "
                    f"{Fore.RED}MISSING{Fore.RESET} {list_cyanize(missing)}, "
                    f"{Fore.BLUE}available {list_cyanize(needed - missing)}"
                    f"{Fore.RESET}"
                )

        async with asyncio.TaskGroup() as tg:
            tg.create_task(mon(), name="ik_monitor_fk_input")
            tg.create_task(good_display(), name="ik_monitor_fk_good_display")
            tg.create_task(bad_display(), name="ik_monitor_fk_bad_display")

    def now(self) -> Time:
        return Time(nano=time.time_ns())
