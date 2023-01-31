from __future__ import annotations

import os
import warnings
from dataclasses import dataclass, field
from abc import ABC, abstractmethod
from .xml import XMLElement, beautify
from .control_mode import ControlMode
from .controller import GravityCompController, JointPositionController
from .control_switch import ControlSwitch
from .jump_condition import JumpCondition
from .sensor import ClockSensor
from .softhand import SofthandController

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

        # create the softhand controller
        self.softhand = SofthandController()

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

        # run the softhand controller
        self.softhand.run()

        # send the ha
        update = rospy.ServiceProxy(topic, UpdateHybridAutomaton)
        update(self.xml(indent=0))

    def create_grasp_controller(self, name: str, pose: list[float], time: float = 1.0):
        # setup a grasp trigger
        self.softhand.register_grasp(name, pose, time)

        # create a gravity comp controller for the time of grasping
        return JointPositionController(name=name,
            goal=[0, 0, 0, 0, 0, 0, 0],
            goal_is_relative=True,
            completion_times=time)
