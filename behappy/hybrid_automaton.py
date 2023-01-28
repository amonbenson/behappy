from dataclasses import dataclass, field
from .xml import XMLElement, beautify
from .control_mode import ControlMode
from .controller import GravityCompController


@dataclass
class HybridAutomaton(XMLElement):
    current_control_mode: str = None

    def __init__(self):
        self._children = []

        # add the default sink controller
        self.add(ControlMode.from_controllers(GravityCompController(name='finished')))
        self.current_control_mode = 'finished'

    def xml(self, indent: int = 0):
        # call the default xml serializer
        xml = super().xml()

        # beautify the document
        if indent > 0:
            return beautify(xml, indent=indent)
