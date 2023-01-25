from dataclasses import dataclass
from ha_element import HAElement
from controller import Controller


def decapitalize(string: str) -> str:
    return string[0].lower() + string[1:]


@dataclass
class ControlSet(HAElement):
    ALLOWED_CHILDREN = [Controller]

    name: str = 'default'

    @property
    def type(self) -> str:
        return decapitalize(self.__class__.__name__)

@dataclass
class PandaControlSet(ControlSet):
    pass
