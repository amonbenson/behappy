from __future__ import annotations

from abc import ABCMeta
from dataclasses import dataclass, field
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

    sensor_shape: tuple = (1, 1)

    @property
    def sensor(self) -> XMLElement:
        # all children are jump conditions
        return self._children[0]

    @staticmethod
    def from_sensor(sensor: Sensor, *kargs, **kwargs) -> JumpCondition:
        # create a dynamically derived class with the correct sensor shape
        shapes = {
            'goal': sensor.SHAPE,
            'norm_weights': sensor.SHAPE
        }
        cls = type('JumpCondition', (JumpCondition,), { 'SHAPES': shapes })

        # instantiate the jump condition
        jump_condition = cls(*kargs, **kwargs, sensor_shape=sensor.SHAPE)
        jump_condition.add(sensor)
        return jump_condition
