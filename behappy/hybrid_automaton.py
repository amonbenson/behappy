from dataclasses import dataclass, field
from .xml import XMLElement, beautify
from .control_mode import ControlMode
from .controller import GravityCompController
from .control_switch import ControlSwitch


@dataclass
class HybridAutomaton(XMLElement):
    name: str
    current_control_mode: str = None
    sink: ControlMode = field(default_factory=lambda: GravityCompController(name='finished'), repr=False)

    def __post_init__(self):
        # add the default sink controller
        self.add(ControlMode.from_controllers(self.sink))
        self.current_control_mode = self.sink.name

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

        return ControlModeBehavior(self, control_mode)


@dataclass
class ControlModeBehavior:
    _ha: HybridAutomaton
    _control_mode: ControlMode

    def when(self, *kargs, **kwargs):
        # create a control switch and link the source
        control_switch = ControlSwitch.from_jump_conitions(*kargs, **kwargs)
        self._ha.add(control_switch)

        return ControlSwitchBehavior(self._ha, self._control_mode, control_switch)

    def then(self, *kargs, **kwargs):
        raise NotImplementedError()

    def finish(self):
        raise NotImplementedError()


@dataclass
class ControlSwitchBehavior:
    _ha: HybridAutomaton
    _source: ControlMode
    _control_switch: ControlSwitch

    def _update_target(self, target: ControlMode):
        # link the source and target and assign a unique name
        self._control_switch.source = self._source.name
        self._control_switch.target = target.name
        self._control_switch.derive_name()

    def then(self, *kargs, **kwargs):
        # link the control switch to the source and target and add it to the ha
        target = ControlMode.from_controllers(*kargs, **kwargs)
        self._update_target(target)

        # add the target control mode
        self._ha.add(target)

        return ControlModeBehavior(self._ha, target)

    def finish(self):
        # use the ha sink as the target
        self._update_target(self._ha.sink)
        return self._ha
