import asyncio
from contextlib import suppress

import asyncio_for_robotics.zenoh as afor
import uvloop
from motion_stack.lvl2.core import IKCore, Lvl2Param

from ms_zenoh_bridge.lvl1_zenoh import PublisherHookJSB, SubscriberHookJSB
from ms_zenoh_bridge.lvl2.lvl2_zenoh import PublisherHookPose, SubscriberHookPose


def ns_topic(namespace: str, topic: str) -> str:
    namespace = namespace.strip("/")
    topic = topic.strip("/")
    return f"{namespace}/{topic}" if namespace else topic


@afor.scoped
async def run_leg(params: Lvl2Param):
    scope = afor.Scope.current()
    core = IKCore(params)

    namespace = params.namespace.strip("/")

    SubscriberHookJSB(core.joint_state_input, ns_topic(namespace, "joint_read"))
    SubscriberHookPose(core.ik_target_input, ns_topic(namespace, "set_ik_target"))

    PublisherHookJSB(core.joint_command_output, ns_topic(namespace, "joint_set"))
    PublisherHookPose(core.fk_output, ns_topic(namespace, "tip_pos"))

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
        uvloop.run(run_leg(params))
