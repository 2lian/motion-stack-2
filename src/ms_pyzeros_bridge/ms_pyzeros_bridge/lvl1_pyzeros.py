import asyncio
import time
from dataclasses import dataclass, field
from typing import Any, AsyncGenerator, Coroutine

import asyncio_for_robotics as afor
import numpy as np
import ros2_pyterfaces.cyclone.all_msgs as cycl_msgs
import ros2_pyterfaces.cyclone.all_srvs as cycl_srvs
import ros2_pyterfaces.cydr.all_msgs as cydr_msgs
from asyncio_for_robotics import BaseSub
from asyncio_for_robotics.core.sub import _AUTO_SCOPE
from motion_stack.lvl1.core import JointCore, JStateBatch
from motion_stack.lvl1.core import Time as TimeMS
from motion_stack.utils.joint_state import JState
from pyzeros import Pub, Server, Sub
from pyzeros._scope import ScopeOwned
from pyzeros.node import Node
from pyzeros.qos import QosProfile
from pyzeros.service_server import Responder
from pyzeros.utils import TopicInfo
from ros2_pyterfaces.cyclone.idl import IdlServiceType, IdlStruct, make_idl_service
from ros2_pyterfaces.cydr import idl as cydr_idl
from ros2_pyterfaces.cydr.all_msgs import Empty, Header, JointState, Time

from .utils import cycl_to_jsb, cydr_to_jsb, js_to_cycl, js_to_cydr


@dataclass
class ReturnJointState_Request(
    IdlStruct, typename="motion_stack_msgs/srv/ReturnJointState_Request"
):
    pass


@dataclass
class ReturnJointState_Response(
    IdlStruct, typename="motion_stack_msgs/srv/ReturnJointState_Response"
):
    js: cycl_msgs.JointState = field(default_factory=cycl_msgs.JointState)


ReturnJointState = make_idl_service(ReturnJointState_Request, ReturnJointState_Response)


class PublisherHookJSB(ScopeOwned):
    def __init__(
        self,
        sub: BaseSub[JStateBatch],
        topic: str | TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState],
        *,
        scope: afor.Scope | None | object = _AUTO_SCOPE,
        session: Node | None = None,
    ):
        """Publishes what's on the sub to ROS 2 using pyzeros.

        Args:
            sub: JStateBatch subscriber whose data should be published
            topic: Topic to publish onto
                - if string, uses cydr with default QoS
                - if TopicInfo, uses it instead (automatically adapts
                                                  between cyclone and cydr)
            scope: afor scope of this object
            session: pyzeros session. Resolved from context if omitted.
        """
        self.filter = set()
        self._base_sub = sub
        if isinstance(topic, str):
            self.topic_info: TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState] = (
                TopicInfo(topic, cydr_msgs.JointState, QosProfile.default())  # type: ignore
            )
        else:
            self.topic_info: TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState] = (
                topic
            )
        assert not isinstance(self.topic_info, str)
        if isinstance(self.topic_info.msg_type, type(cydr_idl.IdlStruct)):
            self._is_cydr = True
            self.topic_info.msg_type.brew()
        else:
            self._is_cydr = False

        self.pyz_pub = Pub(*self.topic_info.as_arg(), session=session, scope=scope)
        self.declare()
        self._init_scope(scope)

    def declare(self):
        self.pyz_pub.declare()
        self._base_sub.asap_callback.append(self._convert_publish)

    def close(self):
        self._base_sub.asap_callback.remove(self._convert_publish)

    def _convert_publish(self, jsb: JStateBatch):
        now = TimeMS.from_parts(nano=time.time_ns())
        if self.filter != set():
            values = [k for k in jsb.values() if k.name in self.filter]
        else:
            values = jsb.values()
        if self._is_cydr:
            msgs = js_to_cydr(values, now)
        else:
            msgs = js_to_cycl(values, now)
        for msg in msgs:
            self.pyz_pub.publish(msg)


class SubscriberHookJSB(ScopeOwned):
    def __init__(
        self,
        sub: BaseSub[JStateBatch],
        topic: str | TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState],
        *,
        scope: afor.Scope | None | object = _AUTO_SCOPE,
        session: Node | None = None,
    ):
        """Publishes what's on the sub to ROS 2 using pyzeros.

        Args:
            sub: JStateBatch subscriber whose data should be published
            topic: Topic to publish onto
                - if string, uses cydr with default QoS
                - if TopicInfo, uses it instead (automatically adapts
                                                  between cyclone and cydr)
            scope: afor scope of this object
            session: pyzeros session. Resolved from context if omitted.
        """
        self.seen = set()
        self._base_sub = sub
        if isinstance(topic, str):
            self.topic_info: TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState] = (
                TopicInfo(topic, cydr_msgs.JointState, QosProfile.default())  # type: ignore
            )
        else:
            self.topic_info: TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState] = (
                topic
            )
        assert not isinstance(self.topic_info, str)
        if isinstance(self.topic_info.msg_type, type(cydr_idl.IdlStruct)):
            self._is_cydr = True
        else:
            self._is_cydr = False

        self.pyz_sub = Sub(*self.topic_info.as_arg(), session=session, scope=scope)
        self._init_scope(scope)
        self._declare()

    def _declare(self):
        self.pyz_sub.asap_callback.append(self._convert_forward)
        self.pyz_sub.declare()

    def close(self):
        self.pyz_sub.asap_callback.remove(self._convert_forward)
        self.pyz_sub.close()

    def _convert_forward(self, msg):
        timestamp = TimeMS.from_parts(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if timestamp.nano == 0:
            timestamp = TimeMS(time.time_ns())
        if self._is_cydr:
            jsb = cydr_to_jsb(msg, timestamp)
        else:
            jsb = cycl_to_jsb(msg, timestamp)
        self.seen.update(jsb.keys())
        self._base_sub._input_data_asyncio(jsb)


class Lvl1Services(ScopeOwned):
    def __init__(
        self,
        joint_core: JointCore,
        *,
        scope: afor.Scope | None | object = _AUTO_SCOPE,
        session: Node | None = None,
    ) -> None:
        self.bg_task: None | asyncio.Task = None
        self.core: JointCore = joint_core

        self.adv_srv = Server(
            ReturnJointState, "advertise_joints", session=session, scope=scope
        )
        self.alive_srv = Server(
            cycl_srvs.Empty, "joint_alive", session=session, scope=scope
        )
        self.iterator = self.adv_srv.listen_reliable(queue_size=0)

        self._init_scope(scope)
        if self._scope is not None:
            self.bg_task = self._scope.task_group.create_task(self.manual_run())

    def close(self):
        if self.bg_task is not None:
            self.bg_task.cancel()
        self.adv_srv.close()
        self.alive_srv.close()

    async def manual_run(self):
        async for responder in self.iterator:
            now = TimeMS.from_parts(nano=time.time_ns())
            jsb = self.core.sensor_pipeline.internal_state.accumulated
            sec = now.to_parts()[0]
            nano = now.to_parts()[1]
            responder.response = ReturnJointState_Response(
                js=cycl_msgs.JointState(
                    header=cycl_msgs.Header(stamp=cycl_msgs.Time(sec, nano)),
                    name=[n.name for n in jsb.values()],  # type: ignore
                    position=[n.position for n in jsb.values()],  # type: ignore
                    velocity=[  # type: ignore
                        (n.velocity if n.velocity is not None else np.nan)
                        for n in jsb.values()
                    ],
                    effort=[  # type: ignore
                        (n.effort if n.effort is not None else np.nan)
                        for n in jsb.values()
                    ],
                )
            )
            responder.send()
