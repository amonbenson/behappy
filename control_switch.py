from dataclasses import dataclass
from ha_element import HAElement
from jump_condition import JumpCondition


@dataclass
class ControlSwitch(HAElement):
    ALLOWED_CHILDREN = [JumpCondition]

    name: str
    source: str = None
    target: str = None
