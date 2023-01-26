from __future__ import annotations

from dataclasses import dataclass
import xml.etree.ElementTree as ET
from xml.dom import minidom
from ha_element import HAElement
from control_mode import ControlMode
from control_set import ControlSet, PandaControlSet
from controller import Controller, GravityCompController
from control_switch import ControlSwitch
import rospy
from hybrid_automaton_msgs.srv import UpdateHybridAutomaton


@dataclass
class HybridAutomaton(HAElement):
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
    
    def find(self, name: str) -> HAElement | None:
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

    def run(self):
        # create the publisher
        update = rospy.ServiceProxy(self.update_topic, UpdateHybridAutomaton)

        # publish the xml
        update(self.xml(indent=0))

    def control_mode(self, *controllers: Controller | str, name: str = None) -> ControlMode:
        # try to get the name from the first karg
        if len(controllers) == 1 and isinstance(controllers[0], str):
            name = controllers[0]
            controllers = []

        # validate
        if len(controllers) == 0 and name is None:
            raise ValueError('No controllers given')
        if len(controllers) > 1 and name is None:
            raise ValueError('Multiple controllers given, but no name')
        
        # try to get the name from the first controller
        if len(controllers) > 0 and name is None:
            name = controllers[0].name

        # if the control mode already exists, return it
        control_mode = self.find(name)
        if control_mode is not None:
            return control_mode

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
