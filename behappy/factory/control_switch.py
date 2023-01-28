from __future__ import annotations

from dataclasses import dataclass, field
from .factory import Factory, FactoryCollection
from .jump_condition import JumpConditionFactory
from ..ha.element import Element
from ..ha.control_switch import ControlSwitch


@dataclass
class ControlSwitchFactory(Factory):
    ALLOWED_CHILDREN = [JumpConditionFactory]
    PRIORITY = 1

    sources: list[str] = field(default_factory=list)
    targets: list[str] = field(default_factory=list)

    def __post_init__(self):
        super().__init__()

    def produce(self, dst: Element):
        # validate fields
        if len(self._children) == 0:
            raise ValueError("No jump conditions given")
        if len(self.sources) == 0:
            raise ValueError("No sources given")
        if len(self.targets) == 0:
            raise ValueError("No targets given")
        
        # iterate through each source and each target
        for source in self.sources:
            for target in self.targets:
                # create a control switch using a derived name from source to target
                name = f"{source}_to_{target}"
                control_switch = ControlSwitch(name=name, source=source, target=target)

                # produce the jump conditions to the control switch
                for jc in self._children:
                    jc.source = self._root.find(source)
                    if jc.source is None:
                        raise ValueError(f"Control switch source '{source}' does not exist.")

                    jc.target = self._root.find(target)
                    if jc.target is None:
                        raise ValueError(f"Control switch target '{target}' does not exist.")

                    jc.produce(control_switch)
                
                # add the control switch to the factory destination
                dst.add(control_switch)

@dataclass
class ControlSwitchFactoryCollection(FactoryCollection):
    ALLOWED_CHILDREN = [ControlSwitchFactory]
    PRIORITY = 1
