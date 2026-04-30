import asyncio
from pathlib import Path

import asyncio_for_robotics as afor
import numpy as np
import pytest
import quaternion as qt

from motion_stack.lvl2.core import IKCore, Lvl2Param
from motion_stack.utils.joint_state import JState
from motion_stack.utils.pose import Pose
from motion_stack.utils.time import Time

HERE = Path(__file__).resolve().parent


@pytest.fixture
def mzero_urdf():
    with open(HERE / "m_zero.urdf") as file:
        return file.read()


@pytest.fixture
async def lvl2_mzero_1(mzero_urdf: str):
    lvl2 = IKCore(
        Lvl2Param(
            urdf=mzero_urdf,
            end_effector_name=1,
            angle_syncer_delta=np.deg2rad(5),
            fk_publish_hz=20,
            monitor_timeout=0.05,
        )
    )

    async with asyncio.TaskGroup() as tg:
        task = tg.create_task(lvl2.run())
        yield lvl2
        task.cancel()


def test_empty_urdf_raises():
    with pytest.raises(ValueError):
        IKCore(Lvl2Param())


async def test_joint_state_updates_angles(lvl2_mzero_1: IKCore):
    payload = {
        "joint1_1": JState("joint1_1", Time(10), 0.1),
        "joint1_2": JState("joint1_2", Time(10), 0.2),
        "joint1_3": JState("joint1_3", Time(10), 0.3),
    }

    lvl2_mzero_1.joint_state_input._input_data_asyncio(payload)

    await asyncio.sleep(0.05)

    assert np.allclose(lvl2_mzero_1.angles, np.array([0.1, 0.2, 0.3]))


async def test_unknown_joint_is_ignored(lvl2_mzero_1: IKCore):
    payload = {
        "joint1_1": JState("joint1_1", Time(10), 0.1),
        "joint1_2": JState("joint1_2", Time(10), 0.2),
        "joint1_3": JState("joint1_3", Time(10), 0.3),
        "bad_joint": JState("bad_joint", Time(10), 999.0),
    }

    lvl2_mzero_1.joint_state_input._input_data_asyncio(payload)

    await asyncio.sleep(0.05)

    assert np.allclose(lvl2_mzero_1.angles, np.array([0.1, 0.2, 0.3]))


async def test_fk_output_after_full_joint_state(lvl2_mzero_1: IKCore):
    payload = {
        "joint1_1": JState("joint1_1", Time(10), 0.1),
        "joint1_2": JState("joint1_2", Time(10), 0.2),
        "joint1_3": JState("joint1_3", Time(10), 0.3),
    }

    out_wait = lvl2_mzero_1.fk_output.wait_for_next()
    lvl2_mzero_1.joint_state_input._input_data_asyncio(payload)

    out = await afor.soft_wait_for(out_wait, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("full joint state did not produce FK output")

    assert isinstance(out, Pose)
    assert out.xyz.shape == (3,)
    assert not np.any(np.isnan(out.xyz))
    assert not np.any(np.isnan(qt.as_float_array(out.quat)))


async def test_no_fk_output_before_full_joint_state(lvl2_mzero_1: IKCore):
    payload = {
        "joint1_1": JState("joint1_1", Time(10), 0.1),
    }

    out_wait = lvl2_mzero_1.fk_output.wait_for_next()
    lvl2_mzero_1.joint_state_input._input_data_asyncio(payload)

    out = await afor.soft_wait_for(out_wait, 0.1)
    assert isinstance(out, TimeoutError)


async def test_command_output_tracking_updates_last_sent(lvl2_mzero_1: IKCore):
    command = {
        "joint1_1": JState("joint1_1", Time(10), 1.1),
        "joint1_2": JState("joint1_2", Time(10), 1.2),
        "joint1_3": JState("joint1_3", Time(10), 1.3),
    }

    lvl2_mzero_1.joint_command_output._input_data_asyncio(command)

    await asyncio.sleep(0.05)

    assert np.allclose(lvl2_mzero_1.last_sent, np.array([1.1, 1.2, 1.3]))


async def test_save_as_last_partial_update(lvl2_mzero_1: IKCore):
    lvl2_mzero_1.last_sent[:] = 0.0

    command = {
        "joint1_2": JState("joint1_2", Time(10), 2.2),
    }

    lvl2_mzero_1.save_as_last(command)

    assert np.allclose(lvl2_mzero_1.last_sent, np.array([0.0, 2.2, 0.0]))


async def test_replace_nan_uses_current_fk(lvl2_mzero_1: IKCore):
    payload = {
        "joint1_1": JState("joint1_1", Time(10), 0.1),
        "joint1_2": JState("joint1_2", Time(10), 0.2),
        "joint1_3": JState("joint1_3", Time(10), 0.3),
    }

    lvl2_mzero_1.joint_state_input._input_data_asyncio(payload)
    await asyncio.sleep(0.05)

    current_fk = lvl2_mzero_1.current_fk()

    target = Pose(
        Time(0),
        np.array([np.nan, np.nan, np.nan]),
        np.quaternion(np.nan, np.nan, np.nan, np.nan),
    )

    fixed = lvl2_mzero_1._replace_nan(target)

    assert np.allclose(fixed.xyz, current_fk.xyz)
    assert fixed.quat == current_fk.quat


async def test_ik_target_produces_joint_command(lvl2_mzero_1: IKCore, monkeypatch):
    payload = {
        "joint1_1": JState("joint1_1", Time(10), 0.0),
        "joint1_2": JState("joint1_2", Time(10), 0.0),
        "joint1_3": JState("joint1_3", Time(10), 0.0),
    }

    lvl2_mzero_1.joint_state_input._input_data_asyncio(payload)
    await asyncio.sleep(0.05)

    def fake_find_next_ik(pose, compute_budget=None, mvt_duration=None):
        return np.array([0.1, 0.2, 0.3], dtype=float)

    monkeypatch.setattr(lvl2_mzero_1, "find_next_ik", fake_find_next_ik)

    target = Pose(
        Time(0),
        np.array([0.0, 0.0, 0.0]),
        np.quaternion(1, 0, 0, 0),
    )

    out_wait = lvl2_mzero_1.joint_command_output.wait_for_next()
    lvl2_mzero_1.ik_target_input._input_data_asyncio(target)

    out = await afor.soft_wait_for(out_wait, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("IK target did not produce joint command output")

    assert set(out.keys()) <= set(lvl2_mzero_1.joint_names)
    assert len(out) > 0
