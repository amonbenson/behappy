from __future__ import annotations

from dataclasses import dataclass
import xml.etree.ElementTree as ET
from xml.dom import minidom
from ha_element import HAElement
from control_mode import ControlMode
from control_set import ControlSet, PandaControlSet
from controller import Controller


@dataclass
class HybridAutomaton(HAElement):
    ALLOWED_CHILDREN = [ControlMode]
    IGNORE_FIELDS = ['default_control_set']

    name: str
    current_control_mode: str = None
    default_control_set: type[ControlSet] = PandaControlSet

    def __post_init__(self):
        # HA is always the root element
        self.root = self

    def xml(self, *, indent: int = 0) -> str:

        # call the automatic xml generation from strings
        xml = super().xml()

        # apply indentation
        if indent > 0:
            doc = minidom.parseString(xml).childNodes[0]
            xml = doc.toprettyxml(indent=' ' * indent).strip()

        return xml

    def start(self, element: HAElement) -> HAElement:
        # add the element to the tree
        # (as this might wrap it in a control mode, we need to catch the returned element)
        element = self.add(element)

        # initialize the current control mode
        self.current_control_mode = element.name

        return element

    def add(self, element: HAElement) -> HAElement:
        control_mode = None

        if isinstance(element, ControlMode):
            # add the control mode directly
            control_mode = element

        elif isinstance(element, ControlSet):
            # wrap the element in a control mode
            control_mode = ControlMode(name=element.name)
            control_mode.add(element)

        elif isinstance(element, Controller):
            # wrap the element in a control set inside a control mode
            control_set = self.default_control_set(name='default')
            control_set.add(element)

            control_mode = ControlMode(name=element.name)
            control_mode.add(control_set)

        # add the control mode
        return super().add(control_mode)
