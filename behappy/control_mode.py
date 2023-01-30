from __future__ import annotations

from dataclasses import dataclass, field
from .controller import Controller
from .xml import XMLElement


def decapitalize(string: str) -> str:
    return string[0].lower() + string[1:]


@dataclass
class ControlSet(XMLElement):
    ELEMENT_NAME = 'ControlSet'

    name: str
    type: str = field(init=False)

    def __post_init__(self):
        # dynamically set the type
        self.type = decapitalize(self.__class__.__name__)

    @property
    def controllers(self) -> list[Controller]:
        # all children are controllers
        return self._children

@dataclass
class PandaControlSet(ControlSet):
    pass

@dataclass
class ControlMode(XMLElement):
    name: str

    def __post_init__(self):
        # create the default control set
        self.add(PandaControlSet('default'))

    @property
    def control_set(self) -> ControlSet:
        # control set is always the one and only direct child
        return self._children[0]

    @staticmethod
    def from_controllers(*controllers: list[Controller], name: str = None) -> ControlMode:
        # validate the controllers
        if len(controllers) == 0:
            raise ValueError("No controllers given")

        # validate the name
        if name is None:
            if len(controllers) > 1:
                raise ValueError("When multiple controllers are defined, a unique name must be assigned")
            else:
                name = controllers[0].name

        # create the control mode and add the controllers
        control_mode = ControlMode(name)
        control_mode.control_set.add(*controllers)
        return control_mode
