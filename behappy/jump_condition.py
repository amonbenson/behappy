from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from .xml import XMLElement
from .sensor import Sensor
import numpy as np

class JumpCriterion(Enum):
    NORM_L1 = 0
    NORM_L2 = 1
    NORM_LINF = 2
    NORM_ROTATION = 3
    NORM_TRANSFORMATION = 4
    THRESH_UPPER_BOUND = 5
    THRESH_LOWER_BOUND = 6


@dataclass
class JumpCondition(XMLElement):
    IGNORE_NONE = True

    goal: np.ndarray
    goal_is_relative: bool = False

    norm_weights: np.ndarray = None
    epsilon: float = 0.01
    negate: bool = None
    jump_criterion: JumpCriterion = JumpCriterion.NORM_L1

    @property
    def sensor(self) -> XMLElement:
        # all children are jump conditions
        return self._children[0]

    @staticmethod
    def from_sensor(sensor: Sensor, *kargs, **kwargs) -> JumpCondition:
        jump_condition = JumpCondition(*kargs, **kwargs)
        jump_condition.SHAPES['goal'] = sensor.SHAPE
        jump_condition.SHAPES['norm_weights'] = sensor.SHAPE

        jump_condition.add(sensor)
        return jump_condition
