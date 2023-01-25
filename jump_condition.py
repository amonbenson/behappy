from dataclasses import dataclass
import numpy as np
from ha_element import HAElement
from sensor import Sensor


@dataclass
class JumpCondition(HAElement):
    ALLOWED_CHILDREN = [Sensor]

    goal: np.ndarray
