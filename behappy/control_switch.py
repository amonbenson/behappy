from __future__ import annotations

from dataclasses import dataclass, field
from .control_block import ControlBlock
from .xml import XMLElement


@dataclass
class ControlSwitch(ControlBlock):
    name: str = None
    source: str = None
    target: str = None

    @property
    def jump_conditions(self) -> list[object]:
        # all children are jump conditions
        return self._children

    def derive_name(self):
        self.name = f"{self.source}_to_{self.target}"

    @staticmethod
    def from_jump_conitions(*jump_conditions: object) -> ControlSwitch:
        # create the control switch
        control_switch = ControlSwitch()
        return control_switch
