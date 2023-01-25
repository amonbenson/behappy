from __future__ import annotations

import numpy as np
from dataclasses import fields
import xml.etree.ElementTree as ET


def convert_attribute(attr: object) -> str:
    if isinstance(attr, str):
        return attr
    elif isinstance(attr, (int, float)):
        return str(attr)
    elif isinstance(attr, bool):
        return '1' if attr else '0'
    elif isinstance(attr, np.ndarray):
        shape = ','.join(map(str, attr.shape))
        data = ', '.join(map(str, attr.flatten()))
        return f"[{shape}]{data}"
    else:
        raise TypeError(f'Attribute of type {type(attr)} is not supported')


class HAElement():
    ELEMENT_NAME: str = None
    EXPORT_FIELDS: list[str] = None
    IGNORE_FIELDS: list[str] = None
    ALLOWED_CHILDREN: list[type[HAElement]] = None
    ALLOW_NONE_ATTRIBUTES: bool = True

    _parent: HAElement = None
    _children: list[HAElement] = None
    _root: HAElement = None

    def xml(self) -> str:
        root = ET.Element(self.ELEMENT_NAME or self.__class__.__name__)

        # append all dataclass fields as attributes
        for field in fields(self):
            if self.EXPORT_FIELDS is not None and field.name not in self.EXPORT_FIELDS:
                continue
            if self.IGNORE_FIELDS is not None and field.name in self.IGNORE_FIELDS:
                continue

            # get the attribute value
            value = getattr(self, field.name)

            if self.ALLOW_NONE_ATTRIBUTES and value is None:
                continue

            # convert it to a string
            attribute = convert_attribute(value)
            root.set(field.name, attribute)

        # append all children as elements
        for child in self._children or []:
            root.append(ET.fromstring(child.xml()))

        return ET.tostring(root).decode('utf-8')

    def set_root(self, element: HAElement):
        if self._root == element:
            return

        # set ha for each child
        self._root = element
        for child in self._children or []:
            child.set_root(element)

    def add_all(self, elements: list[HAElement]) -> HAElement:
        for element in elements:
            self.add(element)
        return elements[-1]

    def add(self, element: HAElement) -> HAElement:
        # make sure the children list exists
        self._children = self._children or []

        # check if the element is allowed
        if self.ALLOWED_CHILDREN is None or not isinstance(element, tuple(self.ALLOWED_CHILDREN)):
            raise TypeError(f'Element of type {element.__class__.__name__} is not allowed as a child of {self.__class__.__name__}')

        # link the element to the tree
        element.set_root(self._root)
        element._parent = self
        self._children.append(element)

        # return the element for chaining
        return element
