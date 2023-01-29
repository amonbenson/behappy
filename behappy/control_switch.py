from __future__ import annotations

from dataclasses import dataclass, field
from .xml import XMLElement
from .jump_condition import JumpCondition


@dataclass
class ControlSwitch(XMLElement):
    source: str
    target: str
    name: str = None

    def __post_init__(self):
        # derive the name from source and target
        if self.name is None:
            self.name = f"{self.source}_to_{self.target}"

    @property
    def jump_conditions(self) -> list[JumpCondition]:
        # all children are jump conditions
        return self._children
