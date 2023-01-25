from dataclasses import dataclass
from .ha_element import HAElement


@dataclass
class Controller(HAElement):
    name: str = 'unnamed_controller'

    @property
    def type(self) -> str:
        return self.__class__.__name__

@dataclass
class HTransformController(Controller):
    pass
