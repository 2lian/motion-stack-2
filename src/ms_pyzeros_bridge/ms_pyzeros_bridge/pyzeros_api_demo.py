import asyncio
from contextlib import suppress
from typing import Tuple

import asyncio_for_robotics.zenoh as afor
import colorama
import pyzeros
import zenoh
from motion_stack.lvl1.joint_api import AsyncJointSyncer, make_delta_time

from ms_pyzeros_bridge.lvl1_pyzeros import PublisherHookJSB, SubscriberHookJSB


@afor.scoped
async def main():
    scope = afor.Scope.current()
    syncer = AsyncJointSyncer()

    pub_found: dict[str, tuple[SubscriberHookJSB, PublisherHookJSB]] = {}

    def token_change(sample):
        try:
            key = str(sample.key_expr)
            if "%joint_read/" not in key:
                return
            for seg in key.split("/"):
                if seg.endswith("%joint_read"):
                    namespace = seg.removesuffix("%joint_read").replace("%", "/")
                    break
            else:
                return

            namespace = namespace.strip("/")
            if sample.kind == zenoh.SampleKind.PUT:
                if namespace in pub_found:
                    return
                print(f"+ {namespace}")
                _sub = SubscriberHookJSB(
                    syncer.sensor_input, f"/{namespace}/joint_read"
                )
                _pub = PublisherHookJSB(
                    syncer.command_output, f"/{namespace}/joint_set"
                )
                _pub.filter = _sub.seen
                pub_found[namespace] = (_sub, _pub)
            elif sample.kind == zenoh.SampleKind.DELETE:
                pair = pub_found.pop(namespace, None)
                if pair is None:
                    return
                print(f"- {namespace}")
                pair[0].close()
                pair[1].close()
        except:
            scope.cancel()

    loop = asyncio.get_event_loop()

    token_watcher = (
        afor.auto_session()
        .liveliness()
        .declare_subscriber(
            "@ros2_lv/**/sensor_msgs::msg::dds_::JointState_/**",
            lambda sample: loop.call_soon_threadsafe(token_change, sample),
            history=True,
        )
    )
    scope.exit_stack.push(lambda *_: token_watcher.undeclare())

    scope.task_group.create_task(syncer.run())

    joints = {f"leg{a}_joint{b}" for a in range(1, 5) for b in range(1, 4)}

    print(f"{colorama.Fore.BLUE}Waiting {colorama.Fore.RESET} for {joints=}")
    await syncer.wait_ready(joints)

    print(f"{colorama.Fore.RED}SYNCER READY :){colorama.Fore.RESET}")
    async for tns in afor.Rate(0.5).listen():
        # joints = syncer._pipeline.internal_state.pull_new().keys()
        joints = syncer.sensor.keys()
        joints = {j for j in joints if "grip" not in j}
        print(joints)
        syncer.speed_safe({j: 0.2 for j in joints}, make_delta_time())


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        with pyzeros.auto_context(node="operator"):
            asyncio.run(main())
