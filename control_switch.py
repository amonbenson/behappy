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

    def xml(self, **kwargs):
        # check if source and target are set
        if self.source is None:
            raise Exception("Control Switch missing source")
        
        if self.target is None:
            raise Exception("Control Switch missing target")

        # run the default xml generator
        return super().xml(**kwargs)
