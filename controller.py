from dataclasses import dataclass
import numpy as np
from ha_element import HAElement


@dataclass
class Controller(HAElement):
    ELEMENT_NAME = 'Controller'

    name: str
    type: str = None

    goal: np.ndarray = None
    goal_is_relative: bool = None

    kp: np.ndarray = None
    kv: np.ndarray = None

    priority: int = None
    completion_times: np.ndarray = None

    v_max: np.ndarray = None
    a_max: np.ndarray = None

    interpolation_type: str = None

    def __post_init__(self):
        # set the type dynamically
        self.type = self.__class__.__name__

@dataclass
class HTransformController(Controller):
    reference_frame: str = None

@dataclass
class GravityCompController(Controller):
    pass
