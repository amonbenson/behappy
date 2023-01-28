from dataclasses import dataclass
from .factory import Factory
from .jump_condition import JumpConditionFactory
from .control_mode import ControlModeCollectionFactory
from ..element import Element
from ..control_switch import ControlSwitch


@dataclass
class ControlSwitchFactory(Factory):
    ALLOWED_CHILDREN = [JumpConditionFactory]
    PRIORITY = 1

    sources: ControlModeCollectionFactory = None
    targets: ControlModeCollectionFactory = None

    def produce(self, dst: Element):
        # validate fields
        if len(self._children) == 0:
            raise ValueError("No jump connditions given")
        if self.sources is None:
            raise ValueError("No sources given")
        if self.targets is None:
            raise ValueError("No targets given")
        
        # iterate through each source and each target
        for source in self.sources._children:
            for target in self.targets._children:
                name = f"{source.name}_to_{target.name}"
                control_switch = ControlSwitch(name=name, source=source.name, target=target.name)

                # produce the jump conditions to the control switch
                for jc in self._children:
                    jc.source = source
                    jc.target = target
                    jc.produce(control_switch)
                
                # add the control switch to the factory destination
                dst.add(control_switch)

@dataclass
class ControlSwitchCollectionFactory(Factory):
    ALLOWED_CHILDREN = [ControlSwitchFactory]
    PRIORITY = 1
