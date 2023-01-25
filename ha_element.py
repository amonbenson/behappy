from __future__ import annotations

from dataclasses import fields
import xml.etree.ElementTree as ET


class HAElement():
    EXPORT_FIELDS: list[str] = None
    IGNORE_FIELDS: list[str] = None
    ALLOWED_CHILDREN: list[type[HAElement]] = None

    _parent: HAElement = None
    _children: list[HAElement] = None

    @property
    def element_name(self) -> str:
        return self.__class__.__name__

    def xml(self) -> str:
        self._children = self._children or []
        root = ET.Element(self.__class__.__name__)

        # append all dataclass fields as attributes
        for field in fields(self):
            if self.EXPORT_FIELDS is not None and field.name not in self.EXPORT_FIELDS:
                continue
            if self.IGNORE_FIELDS is not None and field.name in self.IGNORE_FIELDS:
                continue

            root.set(field.name, str(getattr(self, field.name)))

        # append all children as elements
        for child in self._children:
            root.append(ET.fromstring(child.xml()))

        return ET.tostring(root)

    def add(self, element: HAElement) -> HAElement:
        self._children = self._children or []

        # check if the element is allowed
        if self.ALLOWED_CHILDREN is None or not isinstance(element, tuple(self.ALLOWED_CHILDREN)):
            raise TypeError(f'Element of type {element.__class__.__name__} is not allowed as a child of {self.__class__.__name__}')

        # link the element to the tree
        element._parent = self
        self._children.append(element)

        # return the element for chaining
        return element
