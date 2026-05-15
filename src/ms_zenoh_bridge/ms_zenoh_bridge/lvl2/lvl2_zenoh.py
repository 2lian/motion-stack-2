import asyncio_for_robotics as afor
import asyncio_for_robotics.zenoh as afor_zenoh
import zenoh
from asyncio_for_robotics import BaseSub
from asyncio_for_robotics.core.sub import _AUTO_SCOPE
from motion_stack.utils.pose import Pose

from .utils_pose import pose_to_wire, wire_to_pose


class PublisherHookPose:
    def __init__(
        self,
        sub: BaseSub[Pose],
        topic: str | zenoh.KeyExpr,
        *,
        scope: afor.Scope | None | object = _AUTO_SCOPE,
        session: zenoh.Session | None = None,
    ):
        """Publishes Pose data from a BaseSub to Zenoh using msgspec/msgpack.

        Args:
            sub: BaseSub[Pose] whose data should be published.
            topic: Zenoh key expression to publish onto.
            scope: afor scope of this object.
            session: zenoh session. Resolved from context if omitted.
        """
        self._base_sub = sub
        self.session = afor_zenoh.auto_session(session)
        self.topic = topic
        self.z_pub = self.session.declare_publisher(topic)

        self.declare()
        self._init_scope(scope)

    def _init_scope(self, scope: afor.Scope | None | object = _AUTO_SCOPE):
        if scope is _AUTO_SCOPE:
            scope = afor.Scope.current(default=None)
        self._scope = scope

    def declare(self):
        self._base_sub.asap_callback.append(self._convert_publish)

    def close(self):
        self._base_sub.asap_callback.remove(self._convert_publish)
        self.z_pub.undeclare()

    def _convert_publish(self, pose: Pose):
        msg = pose_to_wire(pose)
        self.z_pub.put(msg)


class SubscriberHookPose:
    def __init__(
        self,
        sub: BaseSub[Pose],
        topic: str | zenoh.KeyExpr,
        *,
        scope: afor.Scope | None | object = _AUTO_SCOPE,
        session: zenoh.Session | None = None,
    ):
        """Subscribes to Zenoh Pose data and forwards it as Motion Stack Pose.

        Args:
            sub: BaseSub[Pose] that should receive decoded Pose data.
            topic: Zenoh key expression to subscribe to.
            scope: afor scope of this object.
            session: zenoh session. Resolved from context if omitted.
        """
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
        pose = wire_to_pose(bytes(sample.payload))
        self._base_sub._input_data_asyncio(pose)
