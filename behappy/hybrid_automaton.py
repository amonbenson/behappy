from __future__ import annotations

import os
import warnings
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from .xml import XMLElement, beautify
from .control_mode import ControlMode
from .controller import Controller, GravityCompController
from .control_switch import ControlSwitch
from .jump_condition import JumpCondition
from .sensor import ClockSensor

if 'BEHAPPY_NO_ROSPY' not in os.environ:
    import rospy
    from hybrid_automaton_msgs.srv import UpdateHybridAutomaton


@dataclass
class HybridAutomaton(XMLElement):
    name: str
    current_control_mode: str = None
    sink: ControlMode = field(default_factory=lambda: GravityCompController(name='finished'), repr=False)

    def __post_init__(self):
        # add the default sink controller
        self.add(ControlMode.from_controllers(self.sink))
        self.current_control_mode = self.sink.name

    def xml(self, indent: int = 0):
        # call the default xml serializer
        xml = super().xml()

        # beautify the document
        if indent > 0:
            xml = beautify(xml, indent=indent)
        
        return xml
    
    def run(self, *, topic: str = '/update_hybrid_automaton'):
        if 'BEHAPPY_NO_ROSPY' in os.environ:
            warnings.warn('Cannot run HybridAutomaton without ROS')
            return
        
        update = rospy.ServiceProxy(topic, UpdateHybridAutomaton)
        update(self.xml(indent=0))
