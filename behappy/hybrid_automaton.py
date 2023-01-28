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

    def start(self, *kargs, **kwargs):
        # create a new control mode and set it as the current one
        control_mode = ControlMode.from_controllers(*kargs, **kwargs)
        self.add(control_mode)
        self.current_control_mode = control_mode.name

        return self

    def then(self, *kargs, **kwargs):
        control_mode = ControlMode.from_controllers(*kargs, **kwargs)

        self.add(control_mode)
        return self
    
    def finish(self):
        return self


@dataclass
class ControlModeBehavior:
    _ha: HybridAutomaton
    _control_mode: ControlMode

    def when(self, *kargs, **kwargs):
        raise NotImplementedError()

    def then(self, *kargs, **kwargs):
        raise NotImplementedError()

    def finish(self):
        return self._ha
