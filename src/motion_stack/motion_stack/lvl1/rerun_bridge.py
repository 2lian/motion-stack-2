import time
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import rerun as rr
import rerun.urdf as rr_urdf
from asyncio_for_robotics import BaseSub

from ..utils.joint_state import JState
from .core import JStateBatch


class RerunJointLogger:
    """Standalone rerun logger initialized with a URDF string.

    Exposes a ``BaseSub[JStateBatch]`` that, when fed joint states,
    logs joint values and updates the URDF transforms in rerun.
    """

    def __init__(
        self,
        urdf: str | None,
        ns: str = "",
        recording: rr.RecordingStream | None = None,
    ) -> None:
        self.ns = ns
        self.assets = f"assets/{ns}".removesuffix("/")
        self.joints = f"joints/{ns}".removesuffix("/")
        self.tf = f"tf/{ns}".removesuffix("/")
        self.recording = recording

        self.sub: BaseSub[JStateBatch] = BaseSub()
        self.sub.asap_callback.append(self._on_data)

        self._temp_dir = TemporaryDirectory(prefix="motion_stack_rerun_bridge_")
        self._urdf_joints: dict[str, rr_urdf.UrdfJoint] = {}
        self._setup_urdf(urdf)

    def _setup_urdf(self, urdf: str) -> None:
        if not urdf:
            return
        urdf_path = Path(self._temp_dir.name) / "robot.urdf"
        urdf_path.write_text(urdf)
        tree = rr_urdf.UrdfTree.from_file_path(
            urdf_path, entity_path_prefix=self.assets
        )
        tree.log_urdf_to_recording(self.recording)
        self._urdf_joints = {j.name: j for j in tree.joints()}

    def _on_data(self, jsb: JStateBatch) -> None:
        rr.set_time("main", timestamp=np.datetime64(time.time_ns(), "ns"))
        self._log_batch(jsb)
        self._log_tf_batch(jsb)

    def _log_batch(self, jsb: JStateBatch) -> None:
        for joint_name, joint_state in jsb.items():
            joint_root = f"{self.joints}/{joint_name}"
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

    def close(self) -> None:
        self.sub.close()
        if self.recording is not None:
            self.recording.flush()
        self._temp_dir.cleanup()
