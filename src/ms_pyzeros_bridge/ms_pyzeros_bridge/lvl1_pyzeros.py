import asyncio
import time
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, AsyncGenerator, AsyncIterator, Coroutine

import asyncio_for_robotics as afor
import colorama
import numpy as np
import ros2_pyterfaces.cyclone.all_msgs as cycl_msgs
import ros2_pyterfaces.cyclone.all_srvs as cycl_srvs
from asyncio_for_robotics import BaseSub
from motion_stack.lvl1 import JointCore, JStateBatch
from motion_stack.lvl1 import Time as TimeMS
from motion_stack.utils.joint_state import JState
from pyzeros.node import Node
from pyzeros.service_server import Responder
from ros2_pyterfaces.cyclone.idl import IdlServiceType, IdlStruct, make_idl_service
from ros2_pyterfaces.cydr.all_msgs import Empty, Header, JointState, Time

# /home/elian/Motion-Stack/src/motion_stack_msgs/srv/ReturnJointState
# 0/leg1/advertise_joints/motion_stack_msgs::srv::dds_::ReturnJointState_/RIHS01_ada8726873e5dff528f797803adbb2a565a371f596224c6a473b6569d9d37b75

# 0/leg1/advertise_joints/motion_stack_msgs::srv::dds_::ReturnJointState_/RIHS01_ada8726873e5dff528f797803adbb2a565a371f596224c6a473b6569d9d37b75
# 0/leg4/advertise_joints/motion_stack_msgs::srv::dds_::ReturnJointState_/RIHS01_ada8726873e5dff528f797803adbb2a565a371f596224c6a473b6569d9d37b75


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


class Bridge:
    def __init__(self, joint_core: JointCore) -> None:
        self.core: JointCore = joint_core
        self.lvl2_input: BaseSub[JStateBatch] = self.core.command_sub
        self.lvl2_ouput: BaseSub[JStateBatch] = self.core.joint_read_output
        self.node = Node("moonbot_zero", namespace="leg1")
        self.joint_read_pub = self.node.create_publisher(JointState, "joint_read")
        self.joint_set_sub = self.node.create_subscriber(JointState, "joint_set")
        self.adv_srv = self.node.create_service(ReturnJointState, "advertise_joints")
        self.alive_srv = self.node.create_service(cycl_srvs.Empty, "joint_alive")
        JointState.brew()

    def run(self) -> Coroutine[Any, Any, None]:
        i_read = self.lvl2_ouput.listen_reliable()
        i_set = self.joint_set_sub.listen_reliable()
        i_adv = self.adv_srv.listen_reliable()

        async def _run():
            async with asyncio.TaskGroup() as tg:
                tg.create_task(self.joint_read_pub.async_bind())
                tg.create_task(self.joint_set_sub.async_bind())
                tg.create_task(self.adv_srv.async_bind())
                tg.create_task(self.node.async_bind())
                tg.create_task(self._publish_jread(i_read))
                tg.create_task(self._listen_jset(i_set))
                tg.create_task(self._srv_respond(i_adv))

        return _run()

    async def _srv_respond(
        self,
        iterator: AsyncGenerator[
            Responder[ReturnJointState_Request, ReturnJointState_Response], None
        ],
    ):
        async for responder in iterator:
            print("got request")
            now = TimeMS.from_parts(nano=time.time_ns())
            jsb = self.core.sensor_pipeline.internal_state.accumulated
            sec = now.to_parts()[0]
            nano = now.to_parts()[1]
            responder.response = ReturnJointState_Response(
                js=cycl_msgs.JointState(
                    header=cycl_msgs.Header(stamp=cycl_msgs.Time(sec, nano)),
                    name=[n.name for n in jsb.values()],
                    position=[n.position for n in jsb.values()],
                )
            )
            responder.send()

    async def _listen_jset(self, iterator: AsyncGenerator[JointState, None]):
        async for msg in iterator:
            # timestamp = TimeMS.from_parts(msg.header.stamp.sec, msg.header.stamp.nanosec)
            # if timestamp == 0:
            timestamp = TimeMS(nano=time.time_ns())
            jsb = {
                name.decode("utf-8"): JState(name.decode("utf-8"), timestamp, position)
                for name, position in zip(msg.name, msg.position)
            }
            __import__('pprint').pprint(jsb)
            self.lvl2_input._input_data_asyncio(jsb)

    async def _publish_jread(self, iterator: AsyncGenerator[JStateBatch, None]):
        async for jsb in iterator:
            now = TimeMS.from_parts(nano=time.time_ns())
            sec = np.int32(now.to_parts()[0])
            nano = np.uint32(now.to_parts()[1])
            msg = JointState(
                header=Header(stamp=Time(sec, nano)),
                name=np.array(
                    [n.name.encode("utf-8") for n in jsb.values()], dtype=np.bytes_
                ),
                position=np.array([n.position for n in jsb.values()], dtype=np.float64),
            )
            self.joint_read_pub.publish(msg)
