from __future__ import annotations

from dataclasses import dataclass, field
from copy import deepcopy
from .element import Element
from .control_set import PandaControlSet
from .control_mode import ControlMode
from .controller import Controller
from .control_switch import ControlSwitch
from .jump_condition import JumpCondition
from .sensor import Sensor


@dataclass
class Factory(Element):
    PRIORITY = 0

    def pre_xml(self):
        # produce as the last step before converting to xml
        self.produce()

        # remove the factory itself from the element tree
        self._parent.remove(self)

    def produce(self, dst: Element):
        self._children = self._children or []

        # produce all children onto the dst
        for child in self._children:
            child.produce(dst)


@dataclass
class ControlModeFactory(Factory):
    ALLOWED_CHILDREN = [Controller]
    PRIORITY = 10

    name: str = None

    def produce(self, dst: Element):
        print(self)
        # extract the name from the first controller
        if self.name is None and len(self._children) == 1:
            self.name = self._children[0].name

        # validate fields
        if self.name is None:
            raise ValueError("No name fiven")
        if len(self._children) == 0:
            raise ValueError("No controllers given")
        
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
class ControlModeCollectionFactory(Factory):
    ALLOWED_CHILDREN = [ControlModeFactory]
    PRIORITY = 10

@dataclass
class JumpConditionFactory(Factory):
    ALLOWED_CHILDREN = []
    PRIORITY = 2

    sensor: Sensor = None
    jump_condition: JumpCondition = None
    source: ControlModeFactory = None
    target: ControlModeFactory = None

    def produce(self, dst: Element):
        # validate fields
        if self.sensor is None:
            raise ValueError("No sensor given")
        if self.jump_condition is None:
            raise ValueError("No jump condition given")
        if self.source is None:
            raise ValueError("No source given")
        if self.target is None:
            raise ValueError("No target given")

        # create deepcopies of the required elements
        sensor = deepcopy(self.sensor)
        jump_condition = deepcopy(self.jump_condition)

        # add the sensor to the jump condition and add that to the factory destination
        jump_condition.add(sensor)
        dst.add(jump_condition)
        

@dataclass
class ControlSwitchFactory(Factory):
    ALLOWED_CHILDREN = [JumpConditionFactory]
    PRIORITY = 1

    sources: ControlModeCollectionFactory = None
    targets: ControlModeCollectionFactory = None

    def produce(self, dst: Element):
        # validate fields
        if len(self._children) == 0:
            raise ValueError("No jump connditions given")
        if self.sources is None:
            raise ValueError("No sources given")
        if self.targets is None:
            raise ValueError("No targets given")
        
        # iterate through each source and each target
        for source in self.sources._children:
            for target in self.targets._children:
                name = f"{source.name}_to_{target.name}"
                control_switch = ControlSwitch(name=name, source=source.name, target=target.name)

                # produce the jump conditions to the control switch
                for jc in self._children:
                    jc.source = source
                    jc.target = target
                    jc.produce(control_switch)
                
                # add the control switch to the factory destination
                dst.add(control_switch)

@dataclass
class ControlSwitchCollectionFactory(Factory):
    ALLOWED_CHILDREN = [ControlSwitchFactory]
    PRIORITY = 1
