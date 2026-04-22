import time
from collections.abc import Iterable

import msgspec
import numpy as np
from motion_stack.lvl1.core import JStateBatch
from motion_stack.utils.joint_state import JState
from motion_stack.utils.time import Time as TimeMS


class WireJState(msgspec.Struct, array_like=True):
    name: str
    position: float | None = None
    velocity: float | None = None
    effort: float | None = None


class WireJStateBatch(msgspec.Struct, array_like=True):
    sec: int
    nano: int
    states: list[WireJState]


_ENCODER = msgspec.msgpack.Encoder()
_DECODER = msgspec.msgpack.Decoder(WireJStateBatch)


def jsb_to_wire(states: Iterable[JState], stamp: TimeMS | None = None) -> bytes:
    if stamp is None:
        stamp = TimeMS.from_parts(nano=time.time_ns())
    sec, nano = stamp.to_parts()
    msg = WireJStateBatch(
        sec=sec,
        nano=nano,
        states=[
            WireJState(
                name=js.name,
                position=_py(js.position),
                velocity=_py(js.velocity),
                effort=_py(js.effort),
            )
            for js in states
        ],
    )
    return _ENCODER.encode(msg)


def wire_to_jsb(payload: bytes) -> JStateBatch:
    msg = _DECODER.decode(payload)
    timestamp = TimeMS.from_parts(msg.sec, msg.nano)
    return {
        s.name: JState(
            s.name,
            timestamp,
            s.position,
            s.velocity,
            s.effort,
        )
        for s in msg.states
    }


def _py(v):
    if isinstance(v, np.generic):
        return v.item()
    return v
