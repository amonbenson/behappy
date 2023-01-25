from __future__ import annotations

from dataclasses import fields
import xml.etree.ElementTree as ET


class HAElement():
    ELEMENT_NAME: str = None
    EXPORT_FIELDS: list[str] = None
    IGNORE_FIELDS: list[str] = None
    ALLOWED_CHILDREN: list[type[HAElement]] = None

    _parent: HAElement = None
    _children: list[HAElement] = None

    def xml(self) -> str:
        root = ET.Element(self.ELEMENT_NAME or self.__class__.__name__)

        # append all dataclass fields as attributes
        for field in fields(self):
            if self.EXPORT_FIELDS is not None and field.name not in self.EXPORT_FIELDS:
                continue
            if self.IGNORE_FIELDS is not None and field.name in self.IGNORE_FIELDS:
                continue

            attribute = getattr(self, field.name)
            #print(field.name, attribute)
            root.set(field.name, str(attribute))

        # append all children as elements
        for child in self._children or []:
            root.append(ET.fromstring(child.xml()))

        return ET.tostring(root)

    def add(self, element: HAElement) -> HAElement:
        # make sure the children list exists
        self._children = self._children or []

        # check if the element is allowed
        if self.ALLOWED_CHILDREN is None or not isinstance(element, tuple(self.ALLOWED_CHILDREN)):
            raise TypeError(f'Element of type {element.__class__.__name__} is not allowed as a child of {self.__class__.__name__}')

        # link the element to the tree
        element._parent = self
        self._children.append(element)

        # return the element for chaining
        return element
