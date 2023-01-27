from dataclasses import dataclass
from .ha_element import HAElement
from .control_set import ControlSet
from .control_switch import ControlSwitch
from .jump_condition import JumpCondition


@dataclass
class ControlMode(HAElement):
    ALLOWED_CHILDREN = [ControlSet]

    name: str

    def when(self, *jump_conditions: JumpCondition) -> ControlSwitch:
        # create a new control switch with no target
        control_switch = ControlSwitch(None, self.name, None)
        control_switch.set_root(self._root)

        # add all the jump conditions
        control_switch.add_all(jump_conditions)

        return control_switch
