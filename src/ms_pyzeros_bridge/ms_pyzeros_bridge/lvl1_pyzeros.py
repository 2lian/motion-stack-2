import asyncio
import time
from copy import deepcopy
from dataclasses import dataclass, field
from pprint import pprint
from typing import Any, AsyncGenerator, AsyncIterator, Coroutine, Generic, TypeVar

import asyncio_for_robotics as afor
import colorama
import numpy as np
import ros2_pyterfaces.cyclone.all_msgs as cycl_msgs
import ros2_pyterfaces.cyclone.all_srvs as cycl_srvs
import ros2_pyterfaces.cydr.all_msgs as cydr_msgs
from asyncio_for_robotics import BaseSub
from motion_stack.lvl1.core import JointCore, JStateBatch
from motion_stack.lvl1.core import Time as TimeMS
from motion_stack.utils.joint_state import JState
from pyzeros.node import Node
from pyzeros.qos import QosProfile
from pyzeros.service_server import Responder
from pyzeros.utils import TopicInfo
from ros2_pyterfaces.cyclone.idl import IdlServiceType, IdlStruct, make_idl_service
from ros2_pyterfaces.cydr import idl as cydr_idl
from ros2_pyterfaces.cydr.all_msgs import Empty, Header, JointState, Time

from .utils import cydr_to_jsb, js_to_cycl, js_to_cydr


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


class PublisherHookJSB:
    def __init__(
        self,
        sub: BaseSub[JStateBatch],
        node: Node,
        topic: str | TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState],
        defer: bool = False,
    ):
        """Publishes what's on the sub to ROS 2 using pyzeros.

        Args:
            sub: JStateBatch subscriber whose data should be pubslished
            node: ROS 2 node
            topic: Topic to publish onto
                - if string, uses cydr with default QoS
                - if Fullon topic info, uses it instead (automatically adapts
                                                         between cyclone and
                                                         cydr)
            defer: TODO
        """
        self._base_sub = sub
        self._node = node
        if isinstance(topic, str):
            self.topic_info: TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState] = (
                TopicInfo(topic, cydr_msgs.JointState, QosProfile.default())
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

        self.pub = self._node.create_publisher(*self.topic_info.as_arg(), defer=defer)

    def run(self) -> Coroutine[Any, Any, None]:
        i_read = self._base_sub.listen_reliable(queue_size=0)
        pub_bind = self.pub.async_bind()

        async def _run():
            async with asyncio.TaskGroup() as tg:
                tg.create_task(pub_bind)
                tg.create_task(self._publish_hook(i_read))

        return _run()

    async def _publish_hook(self, iterator: AsyncGenerator[JStateBatch, None]):
        async for jsb in iterator:
            self.publish(jsb)

    def publish(self, jsb: JStateBatch):
        now = TimeMS.from_parts(nano=time.time_ns())
        if self._is_cydr:
            msgs = js_to_cydr(jsb.values(), now)
        else:
            msgs = js_to_cycl(jsb.values(), now)
        for msg in msgs:
            self.pub.publish(msg)


class SubscriberHookJSB:
    def __init__(
        self,
        sub: BaseSub[JStateBatch],
        node: Node,
        topic: str | TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState],
        defer: bool = False,
    ) -> None:
        self._base_sub = sub
        self._node = node
        if isinstance(topic, str):
            self.topic_info: TopicInfo[cydr_msgs.JointState | cycl_msgs.JointState] = (
                TopicInfo(topic, cydr_msgs.JointState, QosProfile.default())
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

        self.sub = self._node.create_subscriber(*self.topic_info.as_arg(), defer=defer)

    async def _listen_jset(self, iterator: AsyncGenerator[JointState, None]):
        async for msg in iterator:
            timestamp = TimeMS.from_parts(
                msg.header.stamp.sec, msg.header.stamp.nanosec
            )
            if timestamp.nano == 0:
                timestamp = TimeMS(time.time_ns())
            if self._is_cydr:
                jsb = cydr_to_jsb(msg, timestamp)
            else:
                raise NotImplemented("cycl_to_jsb does not exist yet")
            self._base_sub._input_data_asyncio(jsb)

    def run(self) -> Coroutine[Any, Any, None]:
        i_read = self.sub.listen_reliable(queue_size=0)
        sub_bind = self.sub.async_bind()

        async def _run():
            async with asyncio.TaskGroup() as tg:
                tg.create_task(sub_bind)
                tg.create_task(self._listen_jset(i_read))

        return _run()


class Lvl1Services:
    def __init__(self, joint_core: JointCore, node: Node) -> None:
        self.core: JointCore = joint_core
        self.node = node

        self.adv_srv = self.node.create_service(ReturnJointState, "advertise_joints")
        self.alive_srv = self.node.create_service(cycl_srvs.Empty, "joint_alive")

    def run(self) -> Coroutine[Any, Any, None]:
        i_adv = self.adv_srv.listen_reliable()

        async def _run():
            async with asyncio.TaskGroup() as tg:
                tg.create_task(self.adv_srv.async_bind())
                tg.create_task(self.alive_srv.async_bind())
                tg.create_task(self._srv_respond(i_adv))

        return _run()

    async def _srv_respond(
        self,
        iterator: AsyncGenerator[
            Responder[ReturnJointState_Request, ReturnJointState_Response], None
        ],
    ):
        async for responder in iterator:
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
