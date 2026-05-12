from dataclasses import dataclass
from typing import Any

import asyncio_for_robotics as afor
import ros2_pyterfaces.cyclone.all_msgs as cycl_msgs
import ros2_pyterfaces.cydr.all_msgs as cydr_msgs
from asyncio_for_robotics import BaseSub
from asyncio_for_robotics.core.sub import _AUTO_SCOPE
from motion_stack.utils.pose import Pose
from pyzeros import Pub, Sub
from pyzeros._scope import ScopeOwned
from pyzeros.node import Node
from pyzeros.qos import QosProfile
from pyzeros.utils import TopicInfo
from ros2_pyterfaces.cydr import idl as cydr_idl

from .utils_pose import cycl_to_pose, cydr_to_pose, pose_to_cycl, pose_to_cydr


class PublisherHookPose(ScopeOwned):
    def __init__(
        self,
        sub: BaseSub[Pose],
        topic: str | TopicInfo[cydr_msgs.Transform | cycl_msgs.Transform],
        *,
        scope: afor.Scope | None | object = _AUTO_SCOPE,
        session: Node | None = None,
    ):
        """Publishes Pose data from a BaseSub to ROS 2 using pyzeros.

        Args:
            sub: BaseSub[Pose] whose data should be published.
            topic:
                - if string, uses cydr geometry_msgs/Transform with default QoS
                - if TopicInfo, uses it instead and adapts between cyclone/cydr
            scope: afor scope of this object.
            session: pyzeros session. Resolved from context if omitted.
        """
        self._base_sub = sub

        if isinstance(topic, str):
            self.topic_info: TopicInfo[cydr_msgs.Transform | cycl_msgs.Transform] = (
                TopicInfo(topic, cydr_msgs.Transform, QosProfile.default())  # type: ignore
            )
        else:
            self.topic_info: TopicInfo[cydr_msgs.Transform | cycl_msgs.Transform] = (
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
        self.pyz_pub.close()

    def _convert_publish(self, pose: Pose):
        if self._is_cydr:
            msg = pose_to_cydr(pose)
        else:
            msg = pose_to_cycl(pose)

        self.pyz_pub.publish(msg)


class SubscriberHookPose(ScopeOwned):
    def __init__(
        self,
        sub: BaseSub[Pose],
        topic: str | TopicInfo[cydr_msgs.Transform | cycl_msgs.Transform],
        *,
        scope: afor.Scope | None | object = _AUTO_SCOPE,
        session: Node | None = None,
    ):
        """Subscribes to ROS 2 Transform data using pyzeros and forwards as Pose.

        Args:
            sub: BaseSub[Pose] that should receive converted Pose data.
            topic:
                - if string, uses cydr geometry_msgs/Transform with default QoS
                - if TopicInfo, uses it instead and adapts between cyclone/cydr
            scope: afor scope of this object.
            session: pyzeros session. Resolved from context if omitted.
        """
        self._base_sub = sub

        if isinstance(topic, str):
            self.topic_info: TopicInfo[cydr_msgs.Transform | cycl_msgs.Transform] = (
                TopicInfo(topic, cydr_msgs.Transform, QosProfile.default())  # type: ignore
            )
        else:
            self.topic_info: TopicInfo[cydr_msgs.Transform | cycl_msgs.Transform] = (
                topic
            )

        assert not isinstance(self.topic_info, str)

        if isinstance(self.topic_info.msg_type, type(cydr_idl.IdlStruct)):
            self._is_cydr = True
            self.topic_info.msg_type.brew()
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

    def _convert_forward(self, msg: Any):
        if self._is_cydr:
            pose = cydr_to_pose(msg)
        else:
            pose = cycl_to_pose(msg)

        self._base_sub._input_data_asyncio(pose)
