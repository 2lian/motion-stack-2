import asyncio
from contextlib import suppress
from typing import Tuple

import asyncio_for_robotics.zenoh as afor
import colorama
import pyzeros
import zenoh
from asyncio_for_robotics import BaseSub
from motion_stack.lvl1.joint_api import AsyncJointSyncer, make_delta_time
from motion_stack.utils.joint_state import JState, JStateBuffer

from ms_pyzeros_bridge.lvl1_pyzeros import PublisherHookJSB, SubscriberHookJSB


@afor.scoped
async def main():
    scope = afor.Scope.current()
    alive_joints = set()
    syncer = AsyncJointSyncer()
    buffer = JStateBuffer(JState(""))
    syncer.sensor_input.asap_callback.append(buffer.push)
    scope.exit_stack.push(
        lambda *_: syncer.sensor_input.asap_callback.remove(buffer.push)
    )

    pub_found: dict[str, tuple[SubscriberHookJSB, PublisherHookJSB]] = {}
    pub_found_update_sub: BaseSub[int] = BaseSub()

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
            pub_found_update_sub.input_data(1)
        except:
            scope.cancel()

    @afor.scoped
    async def joint_monitor_bkgnd():
        async for _ in afor.Rate(1).listen():
            update_joints()

    async def joint_monitor_fast():
        async for _ in pub_found_update_sub.listen():
            await asyncio.sleep(0.1)
            update_joints()

    def update_joints():
        nonlocal alive_joints
        current: set[str] = set()
        for _sub, _pub in pub_found.values():
            current |= _sub.seen
        if alive_joints == current:
            return
        lost_joints = alive_joints - current
        new_joints = current - alive_joints
        if lost_joints:
            print(
                f"{colorama.Fore.RED}joints lost:{colorama.Fore.RESET} {', '.join(sorted(lost_joints))}"
            )
        if new_joints:
            print(
                f"{colorama.Fore.GREEN}joints found:{colorama.Fore.RESET} {', '.join(sorted(new_joints))}"
            )
        alive_joints = current

    loop = asyncio.get_event_loop()

    token_watcher = (
        afor.auto_session()
        .liveliness()
        .declare_subscriber(
            "@ros2_lv/**/MP/**/sensor_msgs::msg::dds_::JointState_/**",
            lambda sample: loop.call_soon_threadsafe(token_change, sample),
            history=True,
        )
    )
    scope.exit_stack.push(lambda *_: token_watcher.undeclare())

    scope.task_group.create_task(syncer.run())
    scope.task_group.create_task(joint_monitor_fast())
    scope.task_group.create_task(joint_monitor_bkgnd())

    joints = {f"leg{a}_joint{b}" for a in range(1, 2) for b in range(1, 4)}

    print(f"{colorama.Fore.BLUE}Waiting {colorama.Fore.RESET} for {joints=}")
    await syncer.wait_ready(joints)

    print(f"{colorama.Fore.RED}SYNCER READY :){colorama.Fore.RESET}")
    async for tns in afor.Rate(0.5).listen():
        # joints = syncer._pipeline.internal_state.pull_new().keys()
        joints = alive_joints
        joints = {j for j in joints if "grip" not in j}
        syncer.speed_safe({j: 0.2 for j in joints}, make_delta_time())


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        with pyzeros.auto_context(node="operator"):
            asyncio.run(main())
