import asyncio
import numpy as np
import dataclasses
import time
from pathlib import Path
from tempfile import TemporaryDirectory

import rerun as rr
import rerun.urdf as rr_urdf
from asyncio_for_robotics import BaseSub

from ..utils.joint_state import JState
from ..utils.time import Time
from .core import JointCore, JStateBatch


class Lvl1RerunHook:
    def __init__(
        self,
        joint_core: JointCore,
        ns: str = "",
        recording: rr.RecordingStream | None = None,
    ) -> None:
        self.core = joint_core
        self.ns = ns
        self.assets = f"assets/{ns}".removesuffix("/")
        self.joints = f"joints/{ns}".removesuffix("/")
        self.params = f"params/{ns}".removesuffix("/")
        self.tf = f"tf/{ns}".removesuffix("/")
        self.recording = recording
        self._temp_dir = TemporaryDirectory(prefix="motion_stack_lvl1_rerun_")
        self._urdf_joints: dict[str, rr_urdf.UrdfJoint] = {}
        self._setup()

    async def run(self) -> None:
        try:
            async with asyncio.TaskGroup() as tg:
                tg.create_task(
                    self._sub("sensor_sub", self.core.sensor_sub),
                    name="rerun_sensor_sub",
                )
                tg.create_task(
                    self._sub("joint_read_output", self.core.joint_read_output),
                    name="rerun_joint_read_output",
                )
                tg.create_task(
                    self._sub("command_sub", self.core.command_sub),
                    name="rerun_command_sub",
                )
                tg.create_task(
                    self._sub("motor_output", self.core.motor_output),
                    name="rerun_motor_output",
                )
        finally:
            if self.recording is not None:
                self.recording.flush()
            self._temp_dir.cleanup()

    def _setup(self) -> None:
        self.log_params(self.params, self._params_to_dict(self.core.PARAMS))

        if self.core.PARAMS.urdf == "":
            return

        urdf_path = Path(self._temp_dir.name) / "robot.urdf"
        urdf_path.write_text(self.core.PARAMS.urdf)
        urdf = rr_urdf.UrdfTree.from_file_path(
            urdf_path, entity_path_prefix=self.assets
        )
        urdf.log_urdf_to_recording(self.recording)
        self._urdf_joints = {joint.name: joint for joint in urdf.joints()}

    async def _sub(self, stream_name: str, sub: BaseSub[JStateBatch]) -> None:
        async for jsb in sub.listen_reliable():
            rr.set_time("main", timestamp=np.datetime64(time.time_ns(), 'ns'))
            self._log_batch(stream_name, jsb)
            if stream_name == "joint_read_output":
                self._log_tf_batch(jsb)

    def _log_batch(self, stream_name: str, jsb: JStateBatch) -> None:
        for joint_name, joint_state in jsb.items():
            joint_root = f"{self.joints}/{stream_name}/{joint_name}"
            rr.log(
                joint_root,
                rr.AnyValues(
                    name=joint_name,
                    timestamp=(
                        int(joint_state.time.nano)
                        if joint_state.time is not None
                        else None
                    ),
                    position=(
                        float(joint_state.position)
                        if joint_state.position is not None
                        else None
                    ),
                    velocity=(
                        float(joint_state.velocity)
                        if joint_state.velocity is not None
                        else None
                    ),
                    effort=(
                        float(joint_state.effort)
                        if joint_state.effort is not None
                        else None
                    ),
                ),
                recording=self.recording,
            )

    def _log_tf_batch(self, jsb: JStateBatch) -> None:
        for joint_name, joint_state in jsb.items():
            position = joint_state.position
            urdf_joint = self._urdf_joints.get(joint_name)
            if position is None or urdf_joint is None:
                continue
            rr.log(
                self.tf,
                urdf_joint.compute_transform(position, clamp=False),
                recording=self.recording,
            )

    def log_params(self, path: str, value: dict) -> None:
        if not isinstance(value, dict):
            rr.log(
                path,
                rr.AnyValues(drop_untyped_nones=False, value=value),
                static=True,
                recording=self.recording,
            )
            return

        flat = {k: v for k, v in value.items() if not isinstance(v, dict)}
        nested = {k: v for k, v in value.items() if isinstance(v, dict)}

        rr.log(
            path,
            rr.AnyValues(**{k: self._any(v) for k, v in flat.items()}),
            static=True,
            recording=self.recording,
        )

        for key, inner in nested.items():
            self.log_params(f"{path}/{key}", inner)

    def _params_to_dict(self, value):
        if isinstance(value, Time):
            return value.sec
        if dataclasses.is_dataclass(value):
            value = {
                field.name: self._params_to_dict(getattr(value, field.name))
                for field in dataclasses.fields(value)
            }
        if isinstance(value, dict):
            return {str(k): self._params_to_dict(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._params_to_dict(v) for v in value]
        if hasattr(value, "item") and callable(value.item):
            try:
                return value.item()
            except Exception:
                pass
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        return str(value)

    def _any(self, value):
        # return value # works fine
        if value is None:
            return "null"
        if isinstance(value, list):
            if len(value) == 0:
                return "[]"
            return value
        if isinstance(value, str) and value == "":
            return "null"
        return value
