from dataclasses import dataclass, field
from abc import ABC, abstractmethod
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

    def start(self, *controllers):
        # create a new control mode and set it as the current one
        return ControlModeBehavior(self,
            controllers=controllers,
            set_as_current=True)


@dataclass
class Behavior(ABC):
    ha: HybridAutomaton

    @abstractmethod
    def _resolve(self) -> XMLElement:
        raise NotImplementedError()

@dataclass
class JumpConditionBehavior(Behavior):
    pass

@dataclass
class ControlModeBehavior(Behavior):
    controllers: list[Controller] = field(default_factory=list)
    set_as_current: bool = False
    control_mode: ControlMode = field(init=False)

    def __post_init__(self):
        # resolve immediately
        self.control_mode = self._resolve()

    def _resolve(self) -> ControlMode:
        # create the control mode and add all controllers
        control_mode = ControlMode()
        control_mode.add(*self.controllers)

        # add the control mode to the ha
        self.ha.add(control_mode)
        if self.set_as_current:
            self.ha.current_control_mode = control_mode.name

        return control_mode

    def when(self, *kargs, **kwargs):
        # create a control switch and assign the source
        cs_behavior = ControlSwitchBehavior(self.ha,
            source=self.control_mode)
        return cs_behavior

    def then(self, *kargs, **kwargs):
        raise NotImplementedError()

    def finish(self):
        raise NotImplementedError()

@dataclass
class ControlSwitchBehavior(Behavior):
    source: ControlMode = None
    target: ControlMode = None
    jump_condition_behaviors: list[JumpConditionBehavior] = field(default_factory=list)

    def _resolve(self) -> ControlSwitch:
        # set the source and target and assign a unique name
        control_switch = ControlSwitch(
            source=self.source.name,
            target=self.target.name)

        # resolve the jump conditions
        # TODO

        return control_switch

    def _add_jump_condition(self, jump_condition: JumpCondition | JumpConditionBehavior):
        self.jump_condition_behaviors.append(jump_condition)

    def then(self, *controllers):
        # create a target control mode
        cm_behavior = ControlModeBehavior(self.ha, controllers=controllers)

        # add the target control mode
        self.ha.add(target)

        return ControlModeBehavior(self.ha, target)

    def finish(self):
        # use the ha sink as the target
        self._resolve(self.ha.sink)
        return self.ha

    @staticmethod
    def from_source(ha: HybridAutomaton, source: ControlMode) -> ControlSwitch:
        behav = ControlSwitchBehavior(ha, source, None, ControlSwitch(), [])
