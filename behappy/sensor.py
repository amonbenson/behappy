from dataclasses import dataclass, field
from .xml import XMLElement


@dataclass
class Sensor(XMLElement):
    ELEMENT_NAME = 'Sensor'
    SHAPE = None

    type: str = field(init=False)

    def __post_init__(self):
        # dynamically set the type
        self.type = self.__class__.__name__


@dataclass
class ClockSensor(Sensor):
    SHAPE = (1, 1)

@dataclass
class ForceTorqueSensor(Sensor):
    SHAPE = (6, 1)

    port: int = 1
