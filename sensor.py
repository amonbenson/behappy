from dataclasses import dataclass
from ha_element import HAElement


@dataclass
class Sensor(HAElement):
    ELEMENT_NAME = 'Sensor'

    type: str = None

    def __post_init__(self):
        # set the type dynamically
        self.type = self.__class__.__name__


@dataclass
class ClockSensor(Sensor):
    pass
