import operator
from dataclasses import dataclass, asdict
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple, Union, overload

import numpy as np

from .joint_state import JState, MultiJState, multi_to_js_dict

SubShaper = Optional[Callable[[float], float]]


def position_clamp(
    states: Dict[str, JState], limits: Dict[str, Tuple[float, float]]
) -> Dict[str, JState]:
    """Clamps the postion onto the given limits. WARNING inplace operation

    Args:
        states: state of which to clamp the pos
        limits: limits to clamp to

    Returns:

    """
    for name, state in states.items():
        lim = limits.get(name)
        if state.position is None or lim is None:
            continue
        state.position = np.clip(state.position, lim[0], lim[1])
    return states


def operate_sub_shapers(
    shaper1: SubShaper, shaper2: SubShaper, op: Callable[[float, float], float]
) -> SubShaper:
    if shaper1 and shaper2:
        return lambda x: op(shaper1(x), shaper2(x))
    return shaper1 or shaper2


def eggify_shapers(inner: SubShaper, outer: SubShaper) -> SubShaper:
    if inner and outer:
        return lambda x: outer(inner(x))
    return inner or outer


@dataclass
class Shaper:
    """Holds and applies functions to position, velocity and effort fields.

    If None, the indentity is used.
    """

    position: SubShaper = None
    velocity: SubShaper = None
    effort: SubShaper = None

    def _combine(self, other: "Shaper", op: Callable) -> "Shaper":
        return Shaper(
            position=operate_sub_shapers(self.position, other.position, op),
            velocity=operate_sub_shapers(self.velocity, other.velocity, op),
            effort=operate_sub_shapers(self.effort, other.effort, op),
        )

    # Arithmetic operations
    def __add__(self, other: "Shaper") -> "Shaper":
        return self._combine(other, operator.add)

    def __sub__(self, other: "Shaper") -> "Shaper":
        return self._combine(other, operator.sub)

    def __mul__(self, other: "Shaper") -> "Shaper":
        return self._combine(other, operator.mul)

    def __truediv__(self, other: "Shaper") -> "Shaper":
        return NotImplemented
        # return self._combine(other, operator.truediv)

    @overload
    def __call__(self, other: "Shaper") -> "Shaper": ...

    @overload
    def __call__(self, other: JState) -> None: ...

    def __call__(self, other: Union["Shaper", JState]) -> Union["Shaper", None]:
        if isinstance(other, Shaper):
            return Shaper(
                position=eggify_shapers(other.position, self.position),
                velocity=eggify_shapers(other.velocity, self.velocity),
                effort=eggify_shapers(other.effort, self.effort),
            )
        elif isinstance(other, JState):
            apply_shaper(other, self)
            return
        else:
            return NotImplemented


URDFJointName = str
NameMap = Dict[URDFJointName, URDFJointName]
StateMap = Dict[URDFJointName, Shaper]


def reverse_dict(d: Dict) -> Dict:
    return dict(zip(d.values(), d.keys()))


def remap_names(states: List[JState], mapping: NameMap):
    for s in states:
        new_name = mapping.get(s.name)
        if new_name is not None:
            s.name = new_name


def apply_shaper(state: JState, shaper: Shaper):
    for attr, val in asdict(shaper).items():
        sub_shaper: SubShaper = getattr(shaper, attr, None)
        if sub_shaper is None:
            continue
        sub_state: Optional[float] = getattr(state, attr, None)
        if sub_state is None:
            continue
        setattr(state, attr, sub_shaper(sub_state))
        # print(f"{attr}: {sub_state} -> {sub_shaper(sub_state)}")


def shape_states(states: List[JState], mapping: StateMap):
    for s in states:
        shaper = mapping.get(s.name)
        if shaper is not None:
            shaper(s)


"""Apply any function to the input/output of the lvl1 joint core.

This allows for static offset, gain and more. So if your URDF is offsetted from the real robot, this is one solution. It can also change names of joints right before publication/reception, in case you are unhappy with your urdf joint names.

The rempper can apply: 
    - any function: defined as any float->float function.
    - to any attribute: position, velocity or effort .
    - to any joint: Joint names are linked to a function through a dicitonary.
"""


class StateRemapper:
    """Remaps names and applies shaping function to joint state (type: JState).

    This remapper is bi-direcitonal, so data can be mapped then be un-mapped. What thing is mapped/unmapped in the motion stack is simple: the outgoing data is mapped, incomming data is unmapped.

    So, if you apply this to lvl1, as a mapping for lvl0:

        - ``name_map`` will change the name of the joints from to URDF name to another name when **sending to the motors**.
        - ``unname_map`` will change the recieved name to another name when **receiving sensor data**.
        - ``state_map`` applies a :py:class:`.joint_mapper.Shaper` to the joint state when **sending to the motors**.
        - ``unstate_map`` applies a :py:class:`.joint_mapper.Shaper` to the joint state when **recieving from the sensors**.

    Caution:
        - Multiple simultaneous remapping from several keys to one value (name) is possible but use at your own risk.
        - Thus remapping a name, does NOT unbind the original name. So, if mapping incomming j3 to j1, and if incomming j1 is still mapped to j1 (default), data for j1 and j3 will be mapped to j1. This can be undesirable.

    Example, the motion stack is controlling joint 1::

        remap_lvl1 = StateRemapper(
            name_map={"joint1_1": "my_new_joint"}, # "joint1_1" output (usually motor command) is renamed to "my_new_joint"
            unname_map={"another_joint": "joint1_1"}, # "another_joint" intput (usually sensor reading) is renamed to "joint1_1"
            state_map={"joint1_1": Shaper(position=lambda x: x * 2)}, # multiplies command by 2
            unstate_map={"joint1_1": Shaper(position=lambda x: x / 2)}, # divides sensor by 2
        )

    Args:
        name_map: joint name mapping from joint_name -> output_name
        unname_map: joint name unmapping from input_name -> joint_name
        state_map: joint state shaper mapping joint_name -> Shaper_function. Shaper_function will be applied on the output.
        unstate_map: joint state shaper unmapping joint_name -> UnShaper_function. Shaper_function will be applied on the input.
    """

    def __init__(
        self,
        name_map: Dict[str, str] = {},
        unname_map: Optional[Dict[str, str]] = None,
        state_map: Dict[str, Shaper] = {},
        unstate_map: Dict[str, Shaper] = {},
    ) -> None:
        self.name_map: Dict[str, str] = name_map.copy()
        self.unname_map: Dict[str, str] = (
            reverse_dict(self.name_map.copy()) if unname_map is None else unname_map
        )
        self.state_map: Dict[str, Shaper] = state_map.copy()
        self.unstate_map: Dict[str, Shaper] = unstate_map.copy()

    def _namify(self, states: List[JState]):
        remap_names(states, self.name_map)

    def _unnamify(self, states: List[JState]):
        remap_names(states, self.unname_map)

    def _shapify(self, states: List[JState]):
        shape_states(states, self.state_map)

    def _unshapify(self, states: List[JState]):
        shape_states(states, self.unstate_map)

    def map(self, states: MultiJState) -> Dict[str, JState]:
        """Apllies the mapping used when sending"""
        if isinstance(states, dict):
            states = list(states.values())
        elif isinstance(states, JState):
            states = [states]
        else:
            pass
        self._shapify(states)
        self._namify(states)
        return multi_to_js_dict(states)

    def unmap(self, states: MultiJState) -> Dict[str, JState]:
        """Apllies the mapping when receiving"""
        if isinstance(states, dict):
            states = list(states.values())
        elif isinstance(states, JState):
            states = [states]
        else:
            pass
        self._unnamify(states)
        self._unshapify(states)
        return multi_to_js_dict(states)

    def simplify(self, names_to_keep: Iterable[str]) -> "StateRemapper":
        """Eliminates (not in place) all entries whose keys are not in names_to_keep.

        Returns:
            new StateRemapper to use
        """
        # return self
        maps: List[Dict[str, Any]] = [
            self.name_map,
            self.unname_map,
            self.state_map,
            self.unstate_map,
        ]
        new: List[Dict[str, Any]] = [{}, {}, {}, {}]
        for m, mnew in zip(maps, new):
            for k, v in m.items():
                should_be_kept = k in names_to_keep
                if m is self.unname_map:
                    should_be_kept = v in names_to_keep
                if should_be_kept:
                    mnew[k] = v
        return StateRemapper(*new)

    def __call__(self, inner: "StateRemapper") -> "StateRemapper":
        """ok maybe not today"""
        return NotImplemented

    @classmethod
    def empty(cls):
        return cls()


def insert_angle_offset(
    mapper_in: StateRemapper, mapper_out: StateRemapper, offsets: Dict[str, float]
) -> None:
    """Applies an position offsets to an existing StateRemapper shapers.
    the state_map adds the offset, then the unstate_map substracts it (in mapper_out).

    Sub_shapers from mapper_in will overwrite sub_shapers in mapper_out if they are
    affected by offsets.
    mapper_out = mapper_in, may lead to undefined behavior.
    Any function shared between in/out may lead to undefined behavior.
    Use deepcopy() to avoid issues.

    this is a very rough function. feel free to improve

    Args:
        mapper_in: original function map to which offset should be added
        mapper_out: changes will be stored here

    Returns:
        Nothing, the result is stored in the mapper_out.
    """
    # add the offset before executing the original mapping
    # for k, orig_func in mapper_in.state_map.items():
    for k, off_val in offsets.items():
        orig_func = mapper_in.state_map.get(k)
        if orig_func is None:
            orig_func = Shaper()
        # self.parent.pwarn(off_val)
        off_func: SubShaper
        if off_val is not None:
            off_func = lambda x, off=off_val: x + off
        else:
            off_func = None
        off_shaper = Shaper(position=off_func)
        mapper_out.state_map[k] = orig_func(off_shaper)

    # substracts the offset after executing the original unmapping
    # for k, orig_func in mapper_in.unstate_map.items():
    for k, off_val in offsets.items():
        orig_func = mapper_in.unstate_map.get(k)
        if orig_func is None:
            orig_func = Shaper()
        # self.parent.pwarn(off_val)
        off_func: SubShaper
        if off_val is not None:
            off_func = lambda x, off=off_val: x - off
        else:
            off_func = None
        off_shaper = Shaper(position=off_func)
        mapper_out.unstate_map[k] = off_shaper(orig_func)
