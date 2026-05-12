import asyncio
from contextlib import suppress

import asyncio_for_robotics.zenoh as afor
import pyzeros
import uvloop
from motion_stack.lvl2.core import IKCore, Lvl2Param

from ms_pyzeros_bridge.lvl1_pyzeros import PublisherHookJSB, SubscriberHookJSB
from ms_pyzeros_bridge.lvl2.lvl2_pyzeros import PublisherHookPose, SubscriberHookPose


@afor.scoped
async def run_leg(params: Lvl2Param):
    scope = afor.Scope.current()

    core = IKCore(params)

    SubscriberHookJSB(core.joint_state_input, "joint_read")
    SubscriberHookPose(core.ik_target_input, "set_ik_target")

    PublisherHookJSB(core.joint_command_output, "joint_set")
    PublisherHookPose(core.fk_output, "tip_pos")

    scope.task_group.create_task(core.run())

    params.urdf = ""

    await asyncio.Future()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--ms-lvl2-json", required=True)
    args = parser.parse_args()

    params = Lvl2Param.from_json(args.ms_lvl2_json)

    with suppress(KeyboardInterrupt):
        with pyzeros.auto_context(node="lvl2", namespace=params.namespace):
            uvloop.run(run_leg(params))
