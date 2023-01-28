from __future__ import annotations

import os
from dataclasses import dataclass
import xml.etree.ElementTree as ET
from xml.dom import minidom
import warnings
from .element import Element
from .control_mode import ControlMode
from .control_set import ControlSet, PandaControlSet
from .controller import Controller, GravityCompController
from .control_switch import ControlSwitch

if 'BEHAPPY_NO_ROSPY' not in os.environ:
    import rospy
    from hybrid_automaton_msgs.srv import UpdateHybridAutomaton


@dataclass
class HybridAutomaton(Element):
    ALLOWED_CHILDREN = [ControlMode, ControlSwitch]
    IGNORE_FIELDS = ['control_set', 'update_topic']

    name: str
    current_control_mode: str = None
    control_set: type[ControlSet] = PandaControlSet
    update_topic: str = '/update_hybrid_automaton'

    def __post_init__(self):
        # HA is always the root element
        self._root = self

        # create the defaul sink control mode
        self.control_mode(GravityCompController(name='finished'))
        self.current_control_mode = 'finished'
    
    def find(self, name: str) -> Element | None:
        if self._children is None:
            return None

        for child in self._children:
            if child.name == name:
                return child

        return None

    def xml(self, *, indent: int = 0) -> str:

        # call the automatic xml generation from strings
        xml = super().xml()

        # apply indentation
        if indent > 0:
            doc = minidom.parseString(xml).childNodes[0]
            xml = doc.toprettyxml(indent=' ' * indent).strip()

        return xml

    def start(self, *controllers: Controller) -> ControlMode:
        # add the element to the tree
        # (as this might wrap it in a control mode, we need to catch the returned element)
        element = self.control_mode(*controllers)

        # initialize the current control mode
        self.current_control_mode = element.name

        return element

    def control_mode(self, *controllers: Controller | str, name: str = None) -> ControlMode:
        # extract the name if it was passed as the first argument
        if len(controllers) > 0 and isinstance(controllers[0], str):
            name = controllers[0]
            controllers = controllers[1:]

        # if only a name was provided, return an existing control mode
        if len(controllers) == 0 and name is not None:
            control_mode = self.find(name)
            if control_mode is None:
                raise ValueError('No control mode with name {} found'.format(name))
            return control_mode

        # validate the number of controllers
        if len(controllers) == 0:
            raise ValueError('No controllers given')
        if len(controllers) == 1 and name is None:
            name = controllers[0].name
        if len(controllers) > 1 and name is None:
            raise ValueError('Multiple controllers given, but no name')

        # if the control mode already exists, raise an error
        if self.find(name) is not None:
            raise ValueError('Control mode with name {} already exists'.format(name))

        # if only a name was provided, but no exisitng control mode was found, raise an error
        if len(controllers) == 0:
            raise ValueError('No controllers given')

        # create a new control set
        control_set = self.control_set()

        # add all controllers to this set
        for controller in controllers:
            control_set.add(controller)

        # create a new mode where we add the control set to
        control_mode = ControlMode(name=controller.name)
        control_mode.add(control_set)

        return self.add(control_mode)

    def run(self):
        if 'BEHAPPY_NO_ROSPY' in os.environ:
            warnings.warn("Cannot run: BEHAPPY_NO_ROSPY was specified")
            return

        # create the publisher
        update = rospy.ServiceProxy(self.update_topic, UpdateHybridAutomaton)

        # publish the xml
        update(self.xml(indent=0))
