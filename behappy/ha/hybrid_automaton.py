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
    update_topic: str = '/update_hybrid_automaton'

    def __post_init__(self):
        # HA is always the root element
        self._root = self

        # create the defaul sink control mode
        self._add_default_sink()
    
    def _add_default_sink(self):
        # create a gravity comp control mode
        controller = GravityCompController('finished')
        control_set = PandaControlSet('default').add(controller)
        control_mode = ControlMode('finished').add(control_set)

        # add the mode and set it as the current mode
        self.add(control_mode)
        self.current_control_mode = 'finished'

    def find(self, name: str) -> Element | None:
        if self._children is None:
            return None

        for child in self._children:
            if child.name == name:
                return child

        return None

    def xml(self, *, indent: int = 0) -> str:
        # invoke the default xml generator
        xml = super().xml()

        # apply indentation
        if indent > 0:
            doc = minidom.parseString(xml).childNodes[0]
            xml = doc.toprettyxml(indent=' ' * indent).strip()

        return xml

    def run(self):
        if 'BEHAPPY_NO_ROSPY' in os.environ:
            warnings.warn("Cannot run: BEHAPPY_NO_ROSPY was specified")
            return

        # create the publisher
        update = rospy.ServiceProxy(self.update_topic, UpdateHybridAutomaton)

        # publish the xml
        update(self.xml(indent=0))
