from .factory import Factory
from ..ha.hybrid_automaton import HybridAutomaton
from ..ha.control_mode import ControlMode
from ..ha.control_switch import ControlSwitch


class ProducableHybridAutomaton(HybridAutomaton):
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
