from dataclasses import dataclass
import numpy as np
from .ha_element import HAElement


@dataclass
class Controller(HAElement):
    ELEMENT_NAME = 'Controller'

    name: str
    type: str = None

    goal: np.ndarray = None
    goal_is_relative: bool = False

    kp: np.ndarray = None
    kv: np.ndarray = None

    completion_times: np.ndarray = None

    v_max: np.ndarray = None
    a_max: np.ndarray = None

    reinterpolation: bool = False
    priority: int = 0

    operational_frame: str = 'EE'
    reference_frame: str = 'EE'

    interpolation_type: str = None

    def __post_init__(self):
        # set the type dynamically
        self.type = self.__class__.__name__

@dataclass
class JointPositionController(Controller):
    pass

@dataclass
class DisplacementController(Controller):
    if_interpolation: bool = True

@dataclass
class HTransformController(Controller):
    pass

@dataclass
class CartesianVelocityController(Controller):
    control_mode: str = 'cartesian'

    desired_twist: np.ndarray = None
    twist_filter_gain: np.ndarray = None

@dataclass
class GravityCompController(Controller):
    pass
