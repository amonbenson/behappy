from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional
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

    completion_times: Optional[np.ndarray] = 10.0

    v_max: Optional[np.ndarray] = field(default_factory=list)
    a_max: Optional[np.ndarray] = field(default_factory=list)

    reinterpolation: Optional[bool] = False
    priority: Optional[int] = 0

    operational_frame: Optional[str] = None
    reference_frame: Optional[str] = None

    interpolation_type: Optional[str] = None

    def __post_init__(self):
        # dynamically set the type
        self.type = self.__class__.__name__

@dataclass
class JointPositionController(Controller):
    SHAPES = {
        'goal': (7, 1),
        'kp': (7, 1),
        'kv': (7, 1),
        'completion_times': (1, 1),
        'v_max': (0, 0),
        'a_max': (0, 0),
    }

    kp: Optional[np.ndarray] = field(default_factory=lambda: [60, 60, 60, 60, 60, 20, 20])
    kv: Optional[np.ndarray] = field(default_factory=lambda: [30, 30, 30, 20, 20, 20, 10])

@dataclass
class DisplacementController(Controller):
    if_interpolation: Optional[bool] = None

@dataclass
class HTransformController(Controller):
    SHAPES = {
        'goal': (4, 4),
        'kp': (6, 1),
        'kv': (6, 1),
        'completion_times': (1, 1),
        'v_max': (0, 0),
        'a_max': (0, 0),
    }

    kp: Optional[np.ndarray] = field(default_factory=lambda: [500, 500, 500, 100, 100, 100])
    kv: Optional[np.ndarray] = field(default_factory=lambda: [30, 30, 30, 10, 10, 10])

@dataclass
class CartesianVelocityController(Controller):
    SHAPES = {
        'goal': (4, 4),
        'kp': (6, 1),
        'kv': (6, 1),
        'completion_times': (1, 1),
        'v_max': (0, 0),
        'a_max': (0, 0),
    }

    control_mode: Optional[str] = None

    desired_twist: np.ndarray = None
    twist_filter_gain: np.ndarray = None

@dataclass
class GravityCompController(Controller):
    goal: None = field(default=None)
