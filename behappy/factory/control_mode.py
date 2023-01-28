from dataclasses import dataclass
from copy import deepcopy
from .factory import Factory, FactoryCollection
from ..ha.element import Element
from ..ha.control_mode import ControlMode
from ..ha.control_set import PandaControlSet
from ..ha.controller import Controller


@dataclass
class ControlModeFactory(Factory):
    ALLOWED_CHILDREN = [Controller]
    PRIORITY = 10

    name: str = None

    def produce(self, dst: Element):
        # extract the name from the first controller
        if self.name is None and len(self._children) == 1:
            self.name = self._children[0].name

        # validate fields
        if self.name is None:
            raise ValueError("No name fiven")
        if len(self._children) == 0:
            raise ValueError("No controllers given")
        
        # deepcopy the controllers
        controllers = map(deepcopy, self._children)

        # create a new default control set
        control_set = PandaControlSet(name='default')

        # add all controllers to this set
        control_set.add_all(controllers)

        # create a new mode where we add the control set to
        control_mode = ControlMode(name=self.name)
        control_mode.add(control_set)

        # add the control mode to the dst
        dst.add(control_mode)

@dataclass
class ControlModeCollectionFactory(FactoryCollection):
    ALLOWED_CHILDREN = [ControlModeFactory]
    PRIORITY = 10
