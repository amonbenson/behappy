from __future__ import annotations

from dataclasses import dataclass
import xml.etree.ElementTree as ET
from xml.dom import minidom
from ha_element import HAElement
from control_mode import ControlMode
from control_set import ControlSet, PandaControlSet
from controller import Controller
from control_switch import ControlSwitch
import rospy
from hybrid_automaton_msgs.srv import UpdateHybridAutomaton


@dataclass
class HybridAutomaton(HAElement):
    ALLOWED_CHILDREN = [ControlMode, ControlSwitch]
    IGNORE_FIELDS = ['control_set']

    name: str
    current_control_mode: str = None
    control_set: type[ControlSet] = PandaControlSet
    update_topic: str = '/update_hybrid_automaton'

    def __post_init__(self):
        # HA is always the root element
        self._root = self
    
    def find(self, name: str) -> HAElement | None:
        for child in self.children:
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

    def control_mode(self, *controllers: Controller) -> ControlMode:        
        # create a new control set
        control_set = self.control_set()

        # add all controllers to this set
        for controller in controllers:
            control_set.add(controller)

        # create a new mode where we add the control set to
        control_mode = ControlMode(name=controller.name)
        control_mode.add(control_set)

        return self.add(control_mode)
