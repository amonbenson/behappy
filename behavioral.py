import numpy as np
from hybrid_automaton import HybridAutomaton
from control_mode import ControlMode
from controller import Controller
from control_switch import ControlSwitch
from jump_condition import JumpCondition
from sensor import Sensor, ClockSensor


def start(self, *controllers: Controller) -> ControlMode:
    # add the element to the tree
    # (as this might wrap it in a control mode, we need to catch the returned element)
    element = self.control_mode(*controllers)

    # initialize the current control mode
    self.current_control_mode = element.name

    return element

HybridAutomaton.start = start


def reached(self, *kargs, **kwargs):
    # create a jump condition with the given arguments
    jump_condition = JumpCondition(*kargs, **kwargs)
    jump_condition.add(self)

    return jump_condition

Sensor.reached = reached


def when(self, *jump_conditions: JumpCondition) -> ControlSwitch:
    # create a new control switch with no target
    control_switch = ControlSwitch(None, self.name, None)
    control_switch.set_root(self._root)

    # add all the jump conditions
    control_switch.add_all(jump_conditions)

    return control_switch

ControlMode.when = when
ControlMode.when_time_elapsed = lambda self, time: self.when(ClockSensor().reached(np.array([[time]])))


def then(self, target: Controller):
    # create a new control mode from the given target controller
    self._root.control_mode(target)

    # create the corresponding control switch
    self._root.control_switch(self.parent.name, target.name)

    return target

JumpCondition.then = then
