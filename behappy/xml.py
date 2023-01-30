from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field, fields
from enum import Enum
import xml.etree.cElementTree as ET
from xml.dom import minidom
import numpy as np
from .transform import Transform


def beautify(xml: str, *, indent: int = 2) -> str:
    doc = minidom.parseString(xml).childNodes[0]
    return doc.toprettyxml(indent=' ' * indent).strip()

@dataclass
class XMLElement(ABC):
    ELEMENT_NAME = None
    IGNORE_NONE = False
    SHAPES = {}

    _children: list[XMLElement] = field(default_factory=list, init=False, repr=False)

    def add(self, *children: XMLElement) -> XMLElement:
        self._children.extend(children)
        return self

    @property
    def first_child(self):
        return self._children[0] if self._children else None

    @property
    def last_child(self):
        return self._children[-1] if self._children else None

    def serialize_field(self, field: str, value: object) -> str:
        # unwrap transforms
        if isinstance(value, Transform):
            value = value.M

        # force a specific shape
        if field in self.SHAPES and value is not None:
            shape = self.SHAPES[field]
            try:
                value = np.array(value).reshape(shape)
            except ValueError:
                raise ValueError(f"Cannot reshape attribute {self.__class__.__name__}.{field} = {value} to {shape}")

        if isinstance(value, str):
            return str(value)
        elif isinstance(value, bool):
            return '1' if value else '0'
        elif isinstance(value, (int, float)):
            return str(value)
        elif isinstance(value, Enum):
            return value.name
        elif isinstance(value, (np.ndarray, list, tuple)):
            # convert to numpy array
            value = np.array(value)

            # special case for empty arrays
            if value.size == 0:
                return '[0,0]'
            
            # stringify the shape
            shape = ','.join(map(str, value.shape))
            data = ';'.join(map(lambda inner: ','.join(map(str, inner.flatten())), value))
            return f"[{shape}]{data}"
        else:
            return None

    def xml(self):
        element = ET.Element(self.ELEMENT_NAME or self.__class__.__name__)

        # serialize all attribute fields
        for field in fields(self):
            # skip fields which should not be represented
            if not field.repr:
                continue

            # get the field value and convert it to an attribute string
            value = getattr(self, field.name)
            attribute = self.serialize_field(field.name, value)

            if attribute is None:
                if self.IGNORE_NONE:
                    continue
                else:
                    raise ValueError(f"Cannot serialize attribute {self.__class__.__name__}.{field.name} = {value}")

            element.set(field.name, attribute)

        # serialize all children
        for child in self._children:
            element.append(ET.fromstring(child.xml()))

        # return the decoded xml string
        return ET.tostring(element).decode('utf-8')
