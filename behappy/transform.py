from __future__ import annotations

import numpy as np
from transforms3d import euler


class Transform():
    DECIMALS = 5

    def __init__(self, translation: np.ndarray | list[float] = None, rotation: np.ndarray | list[float] = None):
        self.M = np.eye(4)

        # apply initial transform
        if translation is not None:
            self.translate(translation)
        if rotation is not None:
            self.rotate(rotation)
        
    def _round(self):
        self.M = np.round(self.M, self.DECIMALS)
    
    def translate(self, translation: np.ndarray | list[float]):
        self.M[:3, 3] = np.array(translation)
        self._round()

        return self
    
    def rotate(self, rotation: np.ndarray | list[float]):
        self.M[:3, :3] = euler.euler2mat(*np.array(rotation), axes='sxyz')
        self._round()

        return self
