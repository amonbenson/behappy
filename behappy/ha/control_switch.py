from __future__ import annotations

from typing import TYPE_CHECKING
from dataclasses import dataclass
from .element import Element
from .jump_condition import JumpCondition
from .controller import Controller

if TYPE_CHECKING:
    from control_mode import ControlMode


@dataclass
class ControlSwitch(Element):
    ALLOWED_CHILDREN = [JumpCondition]

    name: str
    source: str = None
    target: str = None

    @staticmethod
    def derive_name(source, target):
        return source + '_to_' + target

    def xml(self, **kwargs):
        # check if source and target are set
        if self.source is None:
            raise Exception("Control Switch missing source")
        
        if self.target is None:
            raise Exception("Control Switch missing target")

        # run the default xml generator
        return super().xml(**kwargs)

    def or_when(self, *jump_conditions: JumpCondition) -> ControlSwitch:
        # add all the jump conditions
        self.add_all(jump_conditions)

        return self

    def then(self, *target: Controller | str, **kwargs) -> ControlMode:
        # create a new control mode from the given controller
        target = self._root.control_mode(*target, **kwargs)

        # set the target of the control switch and add it to the ha
        self.target = target.name
        self.name = ControlSwitch.derive_name(self.source, target.name)
        self._root.add(self)

        return target

    def finish(self):
        # return the final gravity comp controller
        return self.then('finished')
