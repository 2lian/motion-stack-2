import time
from collections.abc import Iterable
from typing import Any

import numpy as np
from motion_stack.lvl1.core import JStateBatch
from motion_stack.utils.joint_state import JState
from motion_stack.utils.time import Time


def state_orderinator_3000(allStates: Iterable[JState]) -> list[list[JState]]:
    """Converts a list  of JState to multiple lists containing the same lists
    either empty of equal length."""
    outDic: dict[int, list[JState]] = {}
    for state in allStates:
        idx = 0
        if state.position is not None:
            idx += 2**0
        if state.velocity is not None:
            idx += 2**1
        if state.effort is not None:
            idx += 2**2
        # workingJS = out[idx]

        if not idx in outDic.keys():
            outDic[idx] = [state]
        else:
            outDic[idx].append(state)

    return list(outDic.values())


def js_to_cycl(states: Iterable[JState], stamp: Time | None = None) -> list:
    from ros2_pyterfaces.cyclone import all_msgs as cycl_msgs

    l = []
    if stamp is None:
        stamp = Time(time.time_ns())
    sec = stamp.to_parts()[0]
    nano = stamp.to_parts()[1]
    for jsl in state_orderinator_3000(states):
        l += [
            cycl_msgs.JointState(
                header=cycl_msgs.Header(stamp=cycl_msgs.Time(sec, nano)),
                name=[js.name for js in jsl],
                position=(
                    [js.position for js in jsl] if jsl[0].position is not None else []
                ),
                velocity=(
                    [js.velocity for js in jsl] if jsl[0].velocity is not None else []
                ),
                effort=[js.effort for js in jsl] if jsl[0].effort is not None else [],
            )
        ]
    return l


def js_to_cydr(states: Iterable[JState], stamp: Time | None = None) -> list:
    from ros2_pyterfaces.cydr import all_msgs as cydr_msgs

    l = []
    if stamp is None:
        stamp = Time(time.time_ns())
    sec = np.int32(stamp.to_parts()[0])
    nano = np.uint32(stamp.to_parts()[1])
    for jsl in state_orderinator_3000(states):
        l += [
            cydr_msgs.JointState(
                header=cydr_msgs.Header(stamp=cydr_msgs.Time(sec, nano)),
                name=np.array([js.name.encode("utf-8") for js in jsl], dtype=np.bytes_),
                position=(
                    np.array([js.position for js in jsl], dtype=(np.float64))
                    if jsl[0].position is not None
                    else np.empty(0, dtype=np.float64)
                ),
                velocity=(
                    np.array([js.velocity for js in jsl], dtype=(np.float64))
                    if jsl[0].velocity is not None
                    else np.empty(0, dtype=np.float64)
                ),
                effort=(
                    np.array([js.effort for js in jsl], dtype=(np.float64))
                    if jsl[0].effort is not None
                    else np.empty(0, dtype=np.float64)
                ),
            )
        ]
    return l


def cydr_to_jsb(msg: Any, stamp: Time) -> JStateBatch:
    from ros2_pyterfaces.cydr import all_msgs as cydr_msgs

    msg: cydr_msgs.JointState
    if stamp is None:
        timestamp = Time.from_parts(msg.header.stamp.sec, msg.header.stamp.nanosec)
    else:
        timestamp = stamp

    name = [n.decode("utf-8") for n in msg.name]
    position = msg.position.tolist()
    velocity = msg.velocity.tolist()
    effort = msg.effort.tolist()

    position = position + [None] * (len(name) - len(position))
    velocity = velocity + [None] * (len(name) - len(velocity))
    effort = effort + [None] * (len(name) - len(effort))

    jsb = {
        n: JState(n, timestamp, p, v, e)
        for n, p, v, e in zip(name, position, velocity, effort)
    }
    return jsb

def cycl_to_jsb(msg: Any, stamp: Time) -> JStateBatch:
    from ros2_pyterfaces.cydr import all_msgs as cydr_msgs

    msg: cycl_msgs.JointState
    if stamp is None:
        timestamp = Time.from_parts(msg.header.stamp.sec, msg.header.stamp.nanosec)
    else:
        timestamp = stamp

    name = msg.name
    position = msg.position
    velocity = msg.velocity
    effort = msg.effort

    position = position + [None] * (len(name) - len(position))
    velocity = velocity + [None] * (len(name) - len(velocity))
    effort = effort + [None] * (len(name) - len(effort))

    jsb = {
        n: JState(n, timestamp, p, v, e)
        for n, p, v, e in zip(name, position, velocity, effort)
    }
    return jsb
