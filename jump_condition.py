from dataclasses import dataclass
from ha_element import HAElement
from sensor import Sensor


@dataclass
class JumpCondition(HAElement):
    ALLOWED_CHILDREN = [Sensor]
