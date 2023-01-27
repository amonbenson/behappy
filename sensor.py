from dataclasses import dataclass
from jump_condition import JumpCondition, JumpCriterion
from ha_element import HAElement
import numpy as np


@dataclass
class Sensor(HAElement):
    ELEMENT_NAME = 'Sensor'

    type: str = None

    def __post_init__(self):
        # set the type dynamically
        self.type = self.__class__.__name__

    def reached(self, *kargs, **kwargs):
        # create a jump condition with the given arguments
        jump_condition = JumpCondition(*kargs, **kwargs)
        jump_condition.add(self)

        return jump_condition

# register the sensor as a valid jump condition
JumpCondition.ALLOWED_CHILDREN.append(Sensor)


@dataclass
class ClockSensor(Sensor):
    pass

@dataclass
class ForceTorqueSensor(Sensor):
    port: int = 1


time_elapsed = lambda time: ClockSensor().reached(goal=np.array([[time]]),
    goal_is_relative=True,
    epsilon=0,
    norm_weights=[],
    jump_criterion=JumpCriterion.THRESH_UPPER_BOUND)

frontal_contact = lambda threshold=1, ft_sensor_port=1: ForceTorqueSensor(port=ft_sensor_port).reached(
    jump_criterion=JumpCriterion.THRESH_LOWER_BOUND,
    epsilon=0,
    goal_is_relative=False,
    goal=np.array([[-threshold, 0, 0, 0, 0, 0]]).T,
    norm_weights=np.array([[1, 0, 0, 0, 0, 0]]).T,
    negate=False
)

back_of_hand_contact = lambda threshold=0.2, ft_sensor_port=1: ForceTorqueSensor(port=ft_sensor_port).reached(
    jump_criterion=JumpCriterion.THRESH_UPPER_BOUND,
    epsilon=0,
    goal_is_relative=False,
    goal=np.array([[0, 0, 0, 0, 0, threshold]]).T,
    norm_weights=np.array([[0, 0, 0, 0, 0, 1]]).T,
    negate=False
)
