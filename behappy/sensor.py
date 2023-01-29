from dataclasses import dataclass, field
from .xml import XMLElement


@dataclass
class Sensor(XMLElement):
    ELEMENT_NAME = 'Sensor'

    type: str = field(init=False)

    def __post_init__(self):
        # dynamically set the type
        self.type = self.__class__.__name__


@dataclass
class ClockSensor(Sensor):
    pass

@dataclass
class ForceTorqueSensor(Sensor):
    port: int = 1
