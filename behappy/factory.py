from __future__ import annotations

from dataclasses import dataclass, field
from .element import Element
from .control_set import PandaControlSet
from .control_mode import ControlMode
from .controller import Controller
from .control_switch import Controller
from .jump_condition import JumpCondition
from .sensor import Sensor


@dataclass
class Factory(Element):
    ALLOWED_CHILDREN = ['Factory']
    PRIORITY = 0

    def pre_xml(self):
        # produce as the last step before converting to xml
        self.produce()

        # remove the factory itself from the element tree
        self._parent.remove(self)

    def produce(self, target: Element):
        self._children = self._children or []

        # produce all children onto the target
        for child in self._children:
            child.produce(target)


@dataclass
class ControlModeFactory(Factory):
    PRIORITY = 10

    name: str = None
    controllers: list[Controller] = field(default_factory=list)

    def produce(self, target: Element):
        # extract the name from the first controller
        if self.name is None and len(self.controllers) == 1:
            self.name = self.controllers[0].name

        # make sure all fields exist
        if self.name is None:
            raise ValueError("No name fiven")
        if len(self.controllers) == 0:
            raise ValueError("No controllers given")

        # create a new default control set
        control_set = PandaControlSet(name='default')

        # add all controllers to this set
        for controller in self.controllers:
            control_set.add(controller)

        # create a new mode where we add the control set to
        control_mode = ControlMode(name=controller.name)
        control_mode.add(control_set)

        # add the control mode to the target
        target.add(control_mode)

@dataclass
class ControlModeCollectionFactory(Factory):
    ALLOWED_CHILDREN = [ControlModeFactory]
    PRIORITY = 10

@dataclass
class JumpConditionFactory(Factory):
    PRIORITY = 2

    sensor: Sensor

@dataclass
class ControlSwitchFactory(Factory):
    PRIORITY = 1

    jump_conditions: list[JumpConditionFactory] = field(default_factory=list)
