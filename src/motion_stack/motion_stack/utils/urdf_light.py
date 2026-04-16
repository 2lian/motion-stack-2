"""Lightweight URDF parser using only xml.etree.

Extracts joint names, types, limits, and kinematic chain structure
without importing roboticstoolbox (which pulls in ~130 MB of dependencies).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional, Union


@dataclass
class JointLimit:
    lower: float
    upper: float


@dataclass
class LightJoint:
    """Minimal joint representation compatible with ``get_limit()``."""

    name: str
    joint_type: str
    parent: Optional[str] = None
    child: Optional[str] = None
    limit: Optional[JointLimit] = None


def get_limit(joint: LightJoint) -> tuple[float, float]:
    """Returns (lower, upper) limits for a joint."""
    try:
        lower = joint.limit.lower
        upper = joint.limit.upper
    except AttributeError:
        lower = float("-inf")
        upper = float("inf")
    if upper is None:
        upper = float("inf")
    if lower is None:
        lower = float("-inf")
    return lower, upper


def make_ee(ee_string: Union[None, str, int]) -> Union[None, str, int]:
    if not isinstance(ee_string, str):
        return ee_string
    if ee_string.isdigit():
        return int(ee_string)
    if ee_string in {"ALL", "", "all"}:
        return None
    return ee_string


def parse_urdf_joints(
    urdf: str,
    end_effector_name: Union[str, int, None] = None,
    start_effector_name: Optional[str] = None,
) -> tuple[list[LightJoint], Optional[str], str]:
    """Parse URDF XML and extract joint info for a kinematic chain.

    Drop-in replacement for ``load_set_urdf_raw`` that avoids importing
    roboticstoolbox.  Only extracts what lvl1 needs: joint names, types,
    limits, and chain ordering.

    Returns:
        joints: ordered list of non-fixed joints in the chain
        ee_name: end effector link name, or ``None`` when returning all joints
        base_link_name: name of the root link
    """
    root = ET.fromstring(urdf)

    # -- parse all joints from XML ----------------------------------------
    all_joints: list[LightJoint] = []
    child_to_joint: dict[str, LightJoint] = {}
    child_to_parent: dict[str, str] = {}
    parent_to_children: dict[str, list[str]] = {}

    for jelem in root.findall("joint"):
        name = jelem.get("name", "")
        jtype = jelem.get("type", "fixed")

        parent_elem = jelem.find("parent")
        child_elem = jelem.find("child")
        parent_link = parent_elem.get("link", "") if parent_elem is not None else ""
        child_link = child_elem.get("link", "") if child_elem is not None else ""

        limit_elem = jelem.find("limit")
        if limit_elem is not None:
            lower = float(limit_elem.get("lower", 0))
            upper = float(limit_elem.get("upper", 0))
            limit = JointLimit(lower=lower, upper=upper)
        else:
            limit = None

        j = LightJoint(
            name=name,
            joint_type=jtype,
            parent=parent_link,
            child=child_link,
            limit=limit,
        )
        all_joints.append(j)
        child_to_joint[child_link] = j
        child_to_parent[child_link] = parent_link
        parent_to_children.setdefault(parent_link, []).append(child_link)

    # -- find base link (parent but never a child) ------------------------
    all_parents = {j.parent for j in all_joints if j.parent}
    all_children = {j.child for j in all_joints if j.child}
    root_links = all_parents - all_children
    base_link_name = next(iter(root_links)) if root_links else ""

    if start_effector_name in {None, ""}:
        start_effector_name = None
    start = start_effector_name or base_link_name

    # -- case 1: all joints -----------------------------------------------
    if end_effector_name is None:
        joints = [j for j in all_joints if j.joint_type != "fixed"]
        return joints, None, base_link_name

    # -- case 2: Nth longest chain ----------------------------------------
    if isinstance(end_effector_name, int):
        # DFS from start to find leaves in URDF document order
        leaf_links: list[str] = []
        stack = [start]
        while stack:
            link = stack.pop()
            children = parent_to_children.get(link)
            if children:
                stack.extend(reversed(children))  # reversed so first child is visited first
            else:
                if link != start:
                    leaf_links.append(link)

        paths: list[tuple[int, str]] = []
        for leaf in leaf_links:
            length = 0
            current = leaf
            while current != start and current in child_to_parent:
                length += 1
                current = child_to_parent[current]
            if current == start:
                paths.append((length, leaf))

        paths.sort(key=lambda x: -x[0])
        n = min(end_effector_name, len(paths) - 1)
        ee_link_name = paths[n][1] if paths else base_link_name

    # -- case 3: named end effector ---------------------------------------
    elif start_effector_name == end_effector_name:
        ee_link_name = start
    else:
        if end_effector_name not in all_children and end_effector_name not in all_parents:
            raise ValueError(
                f"end_effector_name={end_effector_name!r} not found in URDF"
            )
        ee_link_name = end_effector_name

    # -- walk backwards collecting non-fixed joints -----------------------
    joints: list[LightJoint] = []
    current = ee_link_name
    while current != start and current in child_to_joint:
        j = child_to_joint[current]
        if j.joint_type != "fixed":
            joints.insert(0, j)
        current = j.parent

    return joints, ee_link_name, base_link_name
