from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from .xml import XMLElement
import numpy as np


@dataclass
class Controller(XMLElement):
    ELEMENT_NAME = 'Controller'
    IGNORE_NONE = True

    name: str
    type: str = field(init=False)

    goal: np.ndarray
    goal_is_relative: Optional[bool] = None

    kp: Optional[np.ndarray] = None
    kv: Optional[np.ndarray] = None

    completion_times: Optional[np.ndarray] = None

    v_max: Optional[np.ndarray] = None
    a_max: Optional[np.ndarray] = None

    reinterpolation: Optional[bool] = None
    priority: Optional[int] = None

    operational_frame: Optional[str] = None
    reference_frame: Optional[str] = None

    interpolation_type: Optional[str] = None

    def __post_init__(self):
        # dynamically set the type
        self.type = self.__class__.__name__

@dataclass
class JointPositionController(Controller):
    pass

@dataclass
class DisplacementController(Controller):
    if_interpolation: Optional[bool] = None

@dataclass
class HTransformController(Controller):
    pass

@dataclass
class CartesianVelocityController(Controller):
    control_mode: Optional[str] = None

    desired_twist: np.ndarray = None
    twist_filter_gain: np.ndarray = None

@dataclass
class GravityCompController(Controller):
    goal: None = field(default=None)
