from __future__ import annotations

from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from .hybrid_automaton import HybridAutomaton
from .xml import XMLElement
from .control_mode import ControlMode
from .control_switch import ControlSwitch
from .controller import Controller
from .jump_condition import JumpCondition, JumpCriterion
from .sensor import ClockSensor
import numpy as np

@dataclass
class Behavior(ABC):
    ha: HybridAutomaton

    @abstractmethod
    def generate(self):
        raise NotImplementedError()

    @abstractmethod
    def apply(self):
        raise NotImplementedError()


@dataclass
class HybridAutomatonBehavior(Behavior):
    def then(self, *controllers: list[Controller], name: str = None):
        # create a new control mode and apply it immediatly
        behav = ControlModeBehavior(self.ha,
            controllers=controllers,
            name=name,
            set_as_current=True)

        # generate, apply and return
        behav.generate()
        behav.apply()
        return behav

    def generate(self):
        raise NotImplementedError()

    def apply(self):
        raise NotImplementedError()


@dataclass
class JumpConditionBehavior(Behavior):
    ha: HybridAutomaton = field(init=False, default=None)
    source: ControlMode = field(init=False, default=None)
    target: ControlMode = field(init=False, default=None)
    switch: ControlSwitch = field(init=False, default=None)
    jump_conditions: list[JumpCondition] = field(default_factory=list)

    def apply(self):
        # add all jump conditions to the parent
        self.switch.add(*self.jump_conditions)


@dataclass
class ControlModeBehavior(Behavior):
    controllers: list[Controller] = field(default_factory=list)
    name: str = None
    set_as_current: bool = False
    control_mode: ControlMode = field(init=False, default=None)

    def generate(self):
        # create the control mode and add all controllers
        self.control_mode = ControlMode.from_controllers(*self.controllers, name=self.name)
    
    def apply(self):
        # add the control mode to the ha
        self.ha.add(self.control_mode)
        if self.set_as_current:
            self.ha.current_control_mode = self.control_mode.name

    def when(self, *jump_conditions):
        # create a control switch and assign the source
        cs_behavior = ControlSwitchBehavior(self.ha,
            jump_conditions=jump_conditions,
            source=self.control_mode)
        return cs_behavior

    def then(self, *controllers):
        return self.when(GoalReached()).then(*controllers)

    def finish(self):
        return self.when(GoalReached()).finish()

@dataclass
class ControlSwitchBehavior(Behavior):
    jump_conditions: list[JumpConditionBehavior] = field(default_factory=list)
    source: ControlMode = None
    target: ControlMode = None
    control_switch: ControlSwitch = field(init=False, default=False)

    def generate(self):
        # set the source and target and assign a unique name
        self.control_switch = ControlSwitch(
            source=self.source.name,
            target=self.target.name)

        # generate all jump conditions
        for jc in self.jump_conditions:
            jc.ha = self.ha
            jc.source = self.source
            jc.target = self.target
            jc.switch = self.control_switch
            jc.generate()
    
    def apply(self):
        # apply all jump conditions
        for jc in self.jump_conditions:
            jc.apply()

        # add the control switch to the ha
        self.ha.add(self.control_switch)

    def then(self, *controllers):
        # create and generate a target control mode
        cm_behavior = ControlModeBehavior(self.ha, controllers=controllers)
        cm_behavior.generate()

        # set the target control mode and resolve
        self.target = cm_behavior.control_mode
        self.generate()

        # apply the control switch and target mode
        self.apply()
        cm_behavior.apply()

        return cm_behavior

    def finish(self):
        # use the ha sink as the target
        self.target = self.ha.sink
        self.generate()
        self.apply()

        return self.ha


def start_ha(*kargs, **kwargs) -> HybridAutomatonBehavior:
    return HybridAutomatonBehavior(HybridAutomaton(*kargs, **kwargs))


@dataclass
class TimeElapsed(JumpConditionBehavior):
    time: float = 0

    def generate(self) -> JumpCondition:
        self.jump_conditions.append(JumpCondition.from_sensor(ClockSensor(),
            goal=self.time,
            goal_is_relative=True,
            epsilon=0,
            norm_weights=1,
            negate=False,
            jump_criterion=JumpCriterion.THRESH_UPPER_BOUND))

@dataclass
class GoalReached(TimeElapsed):
    time: float = field(init=False, default=0)

    def generate(self) -> JumpCondition:
        # extract the time from the source completion duration
        self.time = np.max([np.max(c.completion_times) for c in self.source.control_set.controllers])

        # generate the jump condition
        super().generate()
