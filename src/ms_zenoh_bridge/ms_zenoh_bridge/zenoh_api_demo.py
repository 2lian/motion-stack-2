import asyncio
import time
from contextlib import suppress

import asyncio_for_robotics.zenoh as afor
import colorama
import zenoh
from asyncio_for_robotics import BaseSub
from motion_stack.lvl1.joint_api import AsyncJointSyncer, make_delta_time
from motion_stack.utils.joint_state import JState, JStateBuffer

from ms_zenoh_bridge.lvl1_zenoh import PublisherHookJSB, SubscriberHookJSB

TIMEOUT_S = 2.0


def split_ns_topic(key: str) -> tuple[str, str]:
    key = key.strip("/")
    if "/" not in key:
        return "", key
    namespace, topic = key.rsplit("/", 1)
    return namespace, topic


def ns_topic(namespace: str, topic: str) -> str:
    namespace = namespace.strip("/")
    topic = topic.strip("/")
    return f"{namespace}/{topic}" if namespace else topic


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

    def sample_change(sample: zenoh.Sample):
        key = str(sample.key_expr)
        namespace, topic = split_ns_topic(key)

        if topic != "joint_read":
            return
        if namespace in pub_found:
            return

        # print(sample.kind)
        print(f"+ {namespace}")

        _sub = SubscriberHookJSB(syncer.sensor_input, ns_topic(namespace, "joint_read"))
        _pub = PublisherHookJSB(syncer.command_output, ns_topic(namespace, "joint_set"))
        _pub.filter = _sub.seen
        pub_found[namespace] = (_sub, _pub)
        pub_found_update_sub.input_data(1)

    @afor.scoped
    async def joint_monitor_bkgnd():
        async for _ in afor.Rate(1).listen():
            update_joints()

    async def joint_monitor_fast():
        async for _ in pub_found_update_sub.listen():
            await asyncio.sleep(0.1)
            update_joints()

    async def namespace_gc():
        async for _ in afor.Rate(2).listen():
            now = time.time_ns()
            stale = []

            for namespace, (_sub, _pub) in pub_found.items():
                if (now - _sub.last_sample_ns) / 1e9 > TIMEOUT_S:
                    stale.append(namespace)

            for namespace in stale:
                pair = pub_found.pop(namespace, None)
                if pair is None:
                    continue
                print(f"- {namespace}")
                pair[0].close()
                pair[1].close()
                pub_found_update_sub.input_data(1)

    def update_joints():
        nonlocal alive_joints
        now = time.time_ns()
        current: set[str] = set()

        for _sub, _pub in pub_found.values():
            for joint, ts in _sub.joint_last_seen.items():
                if (now - ts) / 1e9 <= TIMEOUT_S:
                    current.add(joint)

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

    discovery_sub = afor.Sub("**/joint_read")
    discovery_sub.asap_callback.append(sample_change)
    scope.exit_stack.push(lambda *_: discovery_sub.asap_callback.remove(sample_change))
    scope.exit_stack.push(lambda *_: discovery_sub.close())

    scope.task_group.create_task(syncer.run())
    scope.task_group.create_task(joint_monitor_fast())
    scope.task_group.create_task(joint_monitor_bkgnd())
    scope.task_group.create_task(namespace_gc())

    joints = {f"leg{a}_joint{b}" for a in range(1, 2) for b in range(1, 4)}

    print(f"{colorama.Fore.BLUE}Waiting {colorama.Fore.RESET} for {joints=}")
    await syncer.wait_ready(joints)

    print(f"{colorama.Fore.RED}SYNCER READY :){colorama.Fore.RESET}")
    async for _ in afor.Rate(0.5).listen():
        joints = alive_joints
        joints = {j for j in joints if "grip" not in j}
        syncer.speed_safe({j: 0.2 for j in joints}, make_delta_time())


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
