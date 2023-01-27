from dataclasses import dataclass
from enum import Enum
import numpy as np
from .ha_element import HAElement


class JumpCriterion(Enum):
    NORM_L1 = 0
    NORM_L2 = 1
    NORM_LINF = 2
    NORM_ROTATION = 3
    NORM_TRANSFORMATION = 4
    THRESH_UPPER_BOUND = 5
    THRESH_LOWER_BOUND = 6


@dataclass
class JumpCondition(HAElement):
    ALLOWED_CHILDREN = [] # overwritten by sensor module to be [Sensor]. This is a hack to avoid circular imports.

    goal: np.ndarray
    norm_weights: np.ndarray
    goal_is_relative: bool = False
    epsilon: float = 0.01
    negate: bool = False
    jump_criterion: JumpCriterion = JumpCriterion.NORM_L1
