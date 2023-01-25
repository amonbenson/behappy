from dataclasses import dataclass
from ha_element import HAElement
from control_set import ControlSet


@dataclass
class ControlMode(HAElement):
    ALLOWED_CHILDREN = [ControlSet]

    name: str
