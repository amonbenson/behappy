from .factory import Factory
from .control_mode import ControlModeFactory, ControlModeFactoryCollection
from .control_mode import ControlSwitchFactory, ControlSwitchFactoryCollection
from ..ha.hybrid_automaton import HybridAutomaton
from ..ha.control_mode import ControlMode
from ..ha.control_switch import ControlSwitch


class FactoryHybridAutomaton(HybridAutomaton):
    ALLOWED_CHILDREN = [ControlMode, ControlSwitch, Factory]

    def xml(self, *kargs, **kwargs):
        # extract all factories
        factories = filter(lambda el: isinstance(el, Factory), self._children)
        factories = sorted(factories, key=lambda factory: factory.PRIORITY, reverse=True)
        factories = list(factories)

        # remove the factory children
        self._children = filter(lambda el: not isinstance(el, Factory), self._children)
        self._children = list(self._children)

        # invoke the factory production
        for factory in factories:
            factory.produce(self)

        # return the default xml generator
        return super().xml(*kargs, **kwargs)
    
    def start(self, *kargs, **kwargs):
        # create a control mode using the "then" method
        self.then(*kargs, **kwargs)

        # set the initial control mode
        self.current_control_mode = self.last_child._children[0].name

        return self

    def when(self, *jump_conditions):
        if not isinstance(self.last_child, ControlModeFactoryCollection):
            raise TypeError("Last added item must be a control mode")

        # create the control switches an link the previous item as the source control mode
        sources = self.last_child.item_names()
        control_switches = ControlSwitchFactoryCollection.from_item(sources=sources, children=jump_conditions)
        self.add(control_switches)

        return self
    
    def then(self, *controllers):
        # create a new control mode
        control_modes = ControlModeFactoryCollection.from_item(children=controllers)

        # if the last child was a control switch, set the new control modes as its target
        if isinstance(self.last_child, ControlSwitchFactoryCollection):
            self.last_child.targets = control_modes.item_names()

        self.add(control_modes)

        return self


def hybrid_automaton(*kargs, **kwargs) -> FactoryHybridAutomaton:
    return FactoryHybridAutomaton(*kargs, **kwargs)
