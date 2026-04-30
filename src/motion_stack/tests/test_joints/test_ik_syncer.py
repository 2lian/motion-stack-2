import asyncio
import time

import asyncio_for_robotics as afor
import numpy as np
import pytest
import quaternion as qt

from motion_stack.lvl2.ik_api import AsyncIkSyncer, MultiPose
from motion_stack.utils.pose import Pose, VelPose, XyzQuat
from motion_stack.utils.time import Time


def pose(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    quat=None,
) -> Pose:
    if quat is None:
        quat = np.quaternion(1, 0, 0, 0)
    return Pose(Time(0), np.array([x, y, z], dtype=float), quat)


@pytest.fixture
async def syncer():
    s = AsyncIkSyncer(
        interpolation_delta=XyzQuat(10.0, np.deg2rad(10)),
        on_target_delta=XyzQuat(1.0, np.deg2rad(1)),
        batch_time=0.001,
    )

    async with asyncio.TaskGroup() as tg:
        task = tg.create_task(s.run())
        yield s
        task.cancel()


async def test_sensor_input_updates_sensor(syncer: AsyncIkSyncer):
    p = pose(1, 2, 3)

    syncer.sensor_input._input_data_asyncio({1: p})

    await afor.soft_wait_for(syncer.wait_ready({1}), 1)

    assert 1 in syncer.sensor
    assert np.allclose(syncer.sensor[1].xyz, p.xyz)
    assert syncer.sensor[1].quat == p.quat


async def test_wait_ready_blocks_until_requested_limb_exists(syncer: AsyncIkSyncer):
    wait_task = asyncio.create_task(syncer.wait_ready({2}))

    await asyncio.sleep(0.05)
    assert not wait_task.done()

    syncer.sensor_input._input_data_asyncio({1: pose(1, 0, 0)})

    await asyncio.sleep(0.05)
    assert not wait_task.done()

    syncer.sensor_input._input_data_asyncio({2: pose(2, 0, 0)})

    out = await afor.soft_wait_for(wait_task, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("wait_ready did not complete after requested limb arrived")

    assert wait_task.done()


async def test_unsafe_outputs_target_immediately(syncer: AsyncIkSyncer):
    start = pose(0, 0, 0)
    target = pose(100, 0, 0)

    syncer.sensor_input._input_data_asyncio({1: start})
    await afor.soft_wait_for(syncer.wait_ready({1}), 1)

    out_wait = syncer.command_output.wait_for_next()
    fut = syncer.unsafe({1: target})

    out = await afor.soft_wait_for(out_wait, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("unsafe target did not emit command_output")

    assert 1 in out
    assert np.allclose(out[1].xyz, target.xyz)
    assert out[1].quat == target.quat

    # Sensor is still not on target, so future should not be done yet.
    assert not fut.done()


async def test_lerp_outputs_clamped_step(syncer: AsyncIkSyncer):
    start = pose(0, 0, 0)
    target = pose(100, 0, 0)

    syncer.sensor_input._input_data_asyncio({1: start})
    await afor.soft_wait_for(syncer.wait_ready({1}), 1)

    out_wait = syncer.command_output.wait_for_next()
    syncer.lerp({1: target})

    out = await afor.soft_wait_for(out_wait, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("lerp target did not emit command_output")

    assert 1 in out

    # interpolation_delta xyz = 10, so first step should not jump directly to 100.
    assert out[1].xyz[0] > 0
    assert out[1].xyz[0] <= 10.0 + 1e-6
    assert np.allclose(out[1].xyz[1:], [0, 0])


async def test_lerp_future_done_when_sensor_reaches_target(syncer: AsyncIkSyncer):
    start = pose(0, 0, 0)
    target = pose(5, 0, 0)

    syncer.sensor_input._input_data_asyncio({1: start})
    await afor.soft_wait_for(syncer.wait_ready({1}), 1)

    fut = syncer.lerp({1: target})

    # Consume at least one command.
    out = await afor.soft_wait_for(syncer.command_output.wait_for_next(), 1)
    if isinstance(out, TimeoutError):
        pytest.fail("lerp did not emit command")

    # Simulate lvl2/FK sensor reaching target.
    syncer.sensor_input._input_data_asyncio({1: target})

    out = await afor.soft_wait_for(fut, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("lerp future did not complete after sensor reached target")

    assert fut.done()
    assert fut.result() is True


async def test_abs_from_rel_uses_previous_orientation(syncer: AsyncIkSyncer):
    # 90 deg around z. Local x should become world y.
    qz_90 = qt.from_rotation_vector(np.array([0.0, 0.0, np.pi / 2.0]))

    start = pose(10, 20, 30, quat=qz_90)
    rel = pose(1, 0, 0)

    syncer.sensor_input._input_data_asyncio({1: start})
    await afor.soft_wait_for(syncer.wait_ready({1}), 1)

    abs_pose = syncer.abs_from_rel({1: rel})[1]

    assert np.allclose(abs_pose.xyz, np.array([10, 21, 30]), atol=1e-6)


async def test_speed_safe_emits_incremental_target(syncer: AsyncIkSyncer):
    start = pose(0, 0, 0)

    syncer.sensor_input._input_data_asyncio({1: start})
    await afor.soft_wait_for(syncer.wait_ready({1}), 1)

    vp = VelPose(
        time=Time.from_parts(nano=time.time_ns()),
        lin=np.array([10.0, 0.0, 0.0]),
        rvec=np.array([0.0, 0.0, 0.0]),
    )

    out_wait = syncer.command_output.wait_for_next()
    syncer.speed_safe({1: vp}, delta_time=0.1)

    out = await afor.soft_wait_for(out_wait, 1)
    if isinstance(out, TimeoutError):
        pytest.fail("speed_safe did not emit command_output")

    assert 1 in out
    assert np.allclose(out[1].xyz, np.array([1.0, 0.0, 0.0]), atol=1e-6)


async def test_clear_resets_previous_to_sensor(syncer: AsyncIkSyncer):
    start = pose(0, 0, 0)
    target = pose(10, 0, 0)

    syncer.sensor_input._input_data_asyncio({1: start})
    await afor.soft_wait_for(syncer.wait_ready({1}), 1)

    syncer.unsafe({1: target})
    await afor.soft_wait_for(syncer.command_output.wait_for_next(), 1)

    # Now external motion happens.
    external = pose(50, 0, 0)
    syncer.sensor_input._input_data_asyncio({1: external})
    await asyncio.sleep(0.01)

    syncer.clear()

    rel = pose(1, 0, 0)
    abs_pose = syncer.abs_from_rel({1: rel})[1]

    assert np.allclose(abs_pose.xyz, np.array([51, 0, 0]), atol=1e-6)
