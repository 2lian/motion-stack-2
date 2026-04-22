import time

import asyncio_for_robotics as afor
import asyncio_for_robotics.zenoh as afor_zenoh
import zenoh
from asyncio_for_robotics import BaseSub
from asyncio_for_robotics.core.sub import _AUTO_SCOPE
from motion_stack.lvl1.core import JStateBatch
from motion_stack.lvl1.core import Time as TimeMS

from .utils import jsb_to_wire, wire_to_jsb


class PublisherHookJSB:
    def __init__(
        self,
        sub: BaseSub[JStateBatch],
        topic: str | zenoh.KeyExpr,
        *,
        scope: afor.Scope | None | object = _AUTO_SCOPE,
        session: zenoh.Session | None = None,
    ):
        """Publishes what's on the sub to Zenoh using msgspec/msgpack.

        Args:
            sub: JStateBatch subscriber whose data should be published
            topic: Zenoh key expression to publish onto
            scope: afor scope of this object
            session: zenoh session. Resolved from context if omitted.
        """
        self.filter = set()
        self._base_sub = sub
        self.session = afor_zenoh.auto_session(session)
        self.topic = topic
        self.z_pub = self.session.declare_publisher(topic)
        self._init_scope(scope)
        self.declare()

    def _init_scope(self, scope: afor.Scope | None | object = _AUTO_SCOPE):
        if scope is _AUTO_SCOPE:
            scope = afor.Scope.current(default=None)
        self._scope = scope

    def declare(self):
        self._base_sub.asap_callback.append(self._convert_publish)

    def close(self):
        self._base_sub.asap_callback.remove(self._convert_publish)
        self.z_pub.undeclare()

    def _convert_publish(self, jsb: JStateBatch):
        now = TimeMS.from_parts(nano=time.time_ns())
        if self.filter != set():
            values = [k for k in jsb.values() if k.name in self.filter]
        else:
            values = list(jsb.values())
        if not values:
            return
        msg = jsb_to_wire(values, now)
        self.z_pub.put(msg)


class SubscriberHookJSB:
    def __init__(
        self,
        sub: BaseSub[JStateBatch],
        topic: str | zenoh.KeyExpr,
        *,
        scope: afor.Scope | None | object = _AUTO_SCOPE,
        session: zenoh.Session | None = None,
    ):
        """Forwards Zenoh messages into a Motion Stack JStateBatch subscriber.

        Args:
            sub: JStateBatch subscriber to forward decoded data into
            topic: Zenoh key expression to subscribe to
            scope: afor scope of this object
            session: zenoh session. Resolved from context if omitted.
        """
        self.seen = set()
        self.joint_last_seen: dict[str, int] = {}
        self.last_sample_ns = time.time_ns()
        self._base_sub = sub
        self.z_sub = afor_zenoh.Sub(topic, session=session, scope=scope)
        self._init_scope(scope)
        self._declare()

    def _init_scope(self, scope: afor.Scope | None | object = _AUTO_SCOPE):
        if scope is _AUTO_SCOPE:
            scope = afor.Scope.current(default=None)
        self._scope = scope

    def _declare(self):
        self.z_sub.asap_callback.append(self._convert_forward)

    def close(self):
        self.z_sub.asap_callback.remove(self._convert_forward)
        self.z_sub.close()

    def _convert_forward(self, sample: zenoh.Sample):
        jsb = wire_to_jsb(bytes(sample.payload))

        now = time.time_ns()
        self.last_sample_ns = now

        keys = set(jsb.keys())
        self.seen.update(keys)
        for k in keys:
            self.joint_last_seen[k] = now

        self._base_sub._input_data_asyncio(jsb)
