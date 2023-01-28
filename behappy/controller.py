from dataclasses import dataclass
from .xml import XMLElement


@dataclass
class Controller(XMLElement):
    name: str

@dataclass
class GravityCompController(Controller):
    pass
