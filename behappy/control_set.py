from dataclasses import dataclass, field
from .element import Element
from .controller import Controller


def decapitalize(string: str) -> str:
    return string[0].lower() + string[1:]


@dataclass
class ControlSet(Element):
    ELEMENT_NAME = 'ControlSet'
    ALLOWED_CHILDREN = [Controller]

    name: str = 'default'
    type: str = None

    def __post_init__(self):
        # set the type dynamically
        self.type = decapitalize(self.__class__.__name__)


@dataclass
class PandaControlSet(ControlSet):
    pass
