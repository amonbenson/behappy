from dataclasses import dataclass
from .xml import XMLElement


@dataclass
class ControlBlock(XMLElement):
    name: str
