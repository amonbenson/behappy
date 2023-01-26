from dataclasses import dataclass
import numpy as np
from ha_element import HAElement
from sensor import Sensor


@dataclass
class JumpCondition(HAElement):
    ALLOWED_CHILDREN = [Sensor]

    goal: np.ndarray
    norm_weights: np.ndarray
    goal_is_relative: bool = False
    epsilon: float = 0.01
    negate: bool = False
    jump_criterion: str = "NORM_L1"
