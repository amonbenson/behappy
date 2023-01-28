from dataclasses import dataclass
from copy import deepcopy
from typing import TYPE_CHECKING
from .factory import Factory, FactoryCollection
from .control_switch import ControlSwitchFactory, ControlSwitchFactoryCollection
from ..ha.element import Element
from ..ha.control_mode import ControlMode
from ..ha.control_set import PandaControlSet
from ..ha.controller import Controller

if TYPE_CHECKING:
    from .jump_condition import JumpConditionFactory


@dataclass
class ControlModeFactory(Factory):
    ALLOWED_CHILDREN = [Controller]
    PRIORITY = 10

    name: str = None

    def __post_init__(self):
        super().__init__()

    def produce(self, dst: Element):
        # extract the name from the first controller
        if self.name is None and len(self._children) == 1:
            self.name = self._children[0].name

        # validate fields
        if len(self._children) == 0:
            raise ValueError("No controllers given")
        if self.name is None:
            raise ValueError("No name given")
        
        # deepcopy the controllers
        controllers = map(deepcopy, self._children)

        # create a new default control set
        control_set = PandaControlSet(name='default')

        # add all controllers to this set
        control_set.add_all(controllers)

        # create a new mode where we add the control set to
        control_mode = ControlMode(name=self.name)
        control_mode.add(control_set)

        # add the control mode to the dst
        dst.add(control_mode)

@dataclass
class ControlModeFactoryCollection(FactoryCollection):
    ALLOWED_CHILDREN = [ControlModeFactory]
    PRIORITY = 10
