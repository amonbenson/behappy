from dataclasses import dataclass
from copy import deepcopy
from .factory import Factory
from ..ha.element import Element
from ..ha.jump_condition import JumpCondition
from ..ha.control_mode import ControlMode
from ..ha.sensor import Sensor


@dataclass
class JumpConditionFactory(Factory):
    ALLOWED_CHILDREN = []
    PRIORITY = 2

    sensor: Sensor = None
    jump_condition: JumpCondition = None
    source: ControlMode = None
    target: ControlMode = None

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
