from dataclasses import dataclass
from ha_element import HAElement
from jump_condition import JumpCondition
from controller import Controller
from control_mode import ControlMode


@dataclass
class ControlSwitch(HAElement):
    ALLOWED_CHILDREN = [JumpCondition]

    name: str
    source: str = None
    target: str = None

    @staticmethod
    def derive_name(source, target):
        return source + '_to_' + target

    def then(self, target: Controller) -> ControlMode:
        # set the target of the control switch and add it to the ha
        self.target = target.name
        self._root.add(self)

        # create a new control mode from the given controller
        target = self._root.control_mode(target)

        return target

    def xml(self, **kwargs):
        # check if source and target are set
        if self.source is None:
            raise Exception("Control Switch missing source")
        
        if self.target is None:
            raise Exception("Control Switch missing target")

        # run the default xml generator
        return super().xml(**kwargs)
