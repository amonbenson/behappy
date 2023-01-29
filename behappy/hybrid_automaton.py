from __future__ import annotations

from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from .xml import XMLElement, beautify
from .control_mode import ControlMode
from .controller import Controller, GravityCompController
from .control_switch import ControlSwitch
from .jump_condition import JumpCondition
from .sensor import ClockSensor


@dataclass
class HybridAutomaton(XMLElement):
    name: str
    current_control_mode: str = None
    sink: ControlMode = field(default_factory=lambda: GravityCompController(name='finished'), repr=False)

    def __post_init__(self):
        # add the default sink controller
        self.add(ControlMode.from_controllers(self.sink))
        self.current_control_mode = self.sink.name

    def xml(self, indent: int = 0):
        # call the default xml serializer
        xml = super().xml()

        # beautify the document
        if indent > 0:
            return beautify(xml, indent=indent)

    def start(self, *controllers: list[Controller], name: str = None):
        # create a new control mode and set it as the current one
        return ControlModeBehavior(self,
            controllers=controllers,
            name=name,
            set_as_current=True)


@dataclass
class Behavior(ABC):
    ha: HybridAutomaton

    @abstractmethod
    def _resolve(self) -> XMLElement:
        raise NotImplementedError()

@dataclass
class JumpConditionBehavior(Behavior):
    ha: HybridAutomaton = field(init=False)
    source: ControlMode = field(init=False)
    target: ControlMode = field(init=False)

@dataclass
class TimeElapsed(JumpConditionBehavior):
    time: float

    def _resolve(self) -> JumpCondition:
        return JumpCondition.from_sensor(ClockSensor(), goal=self.time)


@dataclass
class ControlModeBehavior(Behavior):
    controllers: list[Controller] = field(default_factory=list)
    name: str = None
    set_as_current: bool = False
    control_mode: ControlMode = field(init=False)

    def __post_init__(self):
        # resolve immediately
        self.control_mode = self._resolve()

    def _resolve(self) -> ControlMode:
        # create the control mode and add all controllers
        control_mode = ControlMode.from_controllers(*self.controllers, name=self.name)
        control_mode.add(*self.controllers)

        # add the control mode to the ha
        self.ha.add(control_mode)
        if self.set_as_current:
            self.ha.current_control_mode = control_mode.name

        return control_mode

    def when(self, *jump_conditions):
        # create a control switch and assign the source
        cs_behavior = ControlSwitchBehavior(self.ha,
            jump_conditions=jump_conditions,
            source=self.control_mode)
        return cs_behavior

    def then(self, *controllers):
        raise NotImplementedError()

    def finish(self):
        raise NotImplementedError()

@dataclass
class ControlSwitchBehavior(Behavior):
    jump_conditions: list[JumpConditionBehavior]
    source: ControlMode = None
    target: ControlMode = None

    def __post_init__(self):
        # setup all jump conditions
        for jc in self.jump_conditions:
            jc.ha = self.ha

    def _resolve(self) -> ControlSwitch:
        # set the source and target and assign a unique name
        control_switch = ControlSwitch(
            source=self.source.name,
            target=self.target.name)

        # resolve all jump conditions
        for jc in self.jump_conditions:
            # setup the jump condition references and resolve it
            jc.source = self.source
            jc.target = self.target
            control_switch.add(jc._resolve())

        self.ha.add(control_switch)

        return control_switch

    def then(self, *controllers):
        # create a target control mode
        cm_behavior = ControlModeBehavior(self.ha, controllers=controllers)

        # set the target control mode and resolve
        self.target = cm_behavior.control_mode
        control_switch = self._resolve()

        return cm_behavior

    def finish(self):
        # use the ha sink as the target
        self.target = self.ha.sink
        self._resolve()

        return self.ha

