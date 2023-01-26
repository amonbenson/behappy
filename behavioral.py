from __future__ import annotations

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

time_elapsed = lambda time: ClockSensor().reached(goal=np.array([[time]]),
    goal_is_relative=True,
    epsilon=0,
    norm_weights=[],
    jump_criterion="THRESH_UPPER_BOUND")


def then(self, *target: Controller | str, **kwargs) -> ControlMode:
    # create a new control mode from the given controller
    target = self._root.control_mode(*target, **kwargs)

    # set the target of the control switch and add it to the ha
    self.target = target.name
    self.name = ControlSwitch.derive_name(self.source, target.name)
    self._root.add(self)

    return target

def end(self):
    # return the final gravity comp controller
    return self.then('finished')

ControlSwitch.then = then
ControlSwitch.end = end
