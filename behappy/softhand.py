from __future__ import annotations

from controller.master_controller import MasterController
from controller.state_collector import StateCollector
from controller.utils import execute_keyframe_interpolation, create_rh3_control_dict
from typing import Optional
from dataclasses import dataclass
import numpy as np
import rospy
from hybrid_automaton_msgs.msg import HAMState
import threading
from .setup import EXPERIMENT_SETUP


class SofthandController:
    def __init__(self, debug: bool = False, state_topic: str = '/ham_state'):
        self._mc = MasterController(EXPERIMENT_SETUP, debug=debug)
        self._sc = StateCollector(self._mc)

        self._triggers = {
            'finished': (None, 1.0) # reset on finished
        }

        # create the state subscriber
        self._state_sub = rospy.Subscriber(state_topic, HAMState, self._state_cb)
        self._prev_state = None
    
    def run(self):
        self._mc.setup()

    def reset(self):
        self._mc.reset()

    def move_to(self, pose: Optional[list[float]], time: float = 0.5):
        # reset instead
        if pose is None:
            self.reset()
            return

        # apply the pose
        control_dict = create_rh3_control_dict(
            kfs=[pose],
            t_initial=time,
            t_duration=[0],
            step_time=0.025,
            absolute=False
        )
        execute_keyframe_interpolation(self._mc, self._sc, control_dict, reset=False)

    def register_grasp(self, name: str, pose: list[float], time: float = 0.5):
        self._triggers[name] = (pose, time)
        print(self._triggers)

    def _state_cb(self, msg: HAMState):
        # get the current state
        state = msg.executing_control_mode_name
        if state == self._prev_state:
            return
        self._prev_state = state

        # get the trigger
        trigger = self._triggers.get(state, None)
        if trigger is None:
            return

        # apply the pose in a new thread
        pose, time = trigger
        self.move_to(pose, time)
    
        # exit rospy
        if state == 'finished':
            rospy.signal_shutdown('HA Finished')
