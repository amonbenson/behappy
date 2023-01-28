from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field, fields
from enum import Enum
import xml.etree.cElementTree as ET
from xml.dom import minidom


def beautify(xml: str, *, indent: int = 2) -> str:
    doc = minidom.parseString(xml).childNodes[0]
    return doc.toprettyxml(indent=' ' * indent).strip()

@dataclass
class XMLElement(ABC):
    ELEMENT_NAME = None
    IGNORE_NONE = False

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

    @staticmethod
    def serialize_attribute(attr: object) -> str:
        if isinstance(attr, str):
            return str(attr)
        elif isinstance(attr, bool):
            return '1' if attr else '0'
        elif isinstance(attr, (int, float)):
            return str(attr)
        elif isinstance(attr, Enum):
            return attr.name
        else:
            return None

    def xml(self):
        element = ET.Element(self.ELEMENT_NAME or self.__class__.__name__)

        # serialize all attribute fields
        for field in fields(self):
            # skip fields which should not be represented
            if not field.repr:
                continue

            # serialize each field as an attribute
            value = getattr(self, field.name)
            attribute = XMLElement.serialize_attribute(value)
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
