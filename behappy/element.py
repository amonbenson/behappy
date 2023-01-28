from __future__ import annotations

import numpy as np
from dataclasses import fields
import xml.etree.ElementTree as ET
from enum import Enum
from .transform import Transform


class Element():
    ELEMENT_NAME: str = None
    EXPORT_FIELDS: list[str] = None
    IGNORE_FIELDS: list[str] = None
    ALLOWED_CHILDREN: list[type[Element]] = None

    _parent: Element = None
    _children: list[Element] = None
    _root: Element = None

    @staticmethod
    def convert_attribute(attr: object) -> str:
        if isinstance(attr, str):
            return attr
        elif isinstance(attr, bool):
            return '1' if attr else '0'
        elif isinstance(attr, (int, float)):
            return str(attr)
        elif isinstance(attr, Enum):
            return attr.name
        elif isinstance(attr, np.ndarray):
            # special case for empty arrays
            if attr.size == 0:
                return '[0,0]'

            shape = ','.join(map(str, attr.shape))
            matrix = attr.reshape(-1, attr.shape[-1])
            data = ';'.join(map(lambda x: ','.join(map(str, x)), matrix))
            return f"[{shape}]{data}"
        elif isinstance(attr, list):
            return self.convert_attribute(np.array(attr))
        elif isinstance(attr, Transform):
            return self.convert_attribute(attr.M)
        else:
            raise TypeError(f'Attribute of type {type(attr)} is not supported')

    def xml(self) -> str:
        # create the root element
        name = self.ELEMENT_NAME or self.__class__.__name__
        root = ET.Element(name)

        # append all dataclass fields as attributes
        for field in fields(self):
            if self.EXPORT_FIELDS is not None and field.name not in self.EXPORT_FIELDS:
                continue
            if self.IGNORE_FIELDS is not None and field.name in self.IGNORE_FIELDS:
                continue

            # get the attribute value
            value = getattr(self, field.name)

            # skip None type attributes (this will instruct the HA to use a default value)
            if value is None:
                continue

            # convert it to a string
            attribute = Element.convert_attribute(value)
            root.set(field.name, attribute)

        # append all children as elements
        for child in self._children or []:
            root.append(ET.fromstring(child.xml()))

        # convert the element into an XML string
        return ET.tostring(root).decode('utf-8')

    def set_root(self, element: Element):
        if self._root == element:
            return

        # set ha for each child
        self._root = element
        for child in self._children or []:
            child.set_root(element)

    def add(self, child: Element) -> Element:
        # make sure the children list exists
        self._children = self._children or []

        # apply the pre function
        self.pre_add(child)

        # check if the child element is allowed
        if self.ALLOWED_CHILDREN is None or not isinstance(child, tuple(self.ALLOWED_CHILDREN)):
            raise TypeError(f'Element of type {child.__class__.__name__} is not allowed as a child of {self.__class__.__name__}')

        # link the element to the HA tree
        child.set_root(self._root)
        child._parent = self
        self._children.append(child)

        # apply the post function
        child = self.post_add(child)

        # return the element for chaining
        return child

    def add_all(self, children: list[Element]) -> Element:
        # add each child
        for child in children:
            self.add(child)
        
        # return the last added child
        return children[-1]

    def remove(self, child: Element) -> Element:
        if not child in self._children:
            raise ValueError(f"Element {child} was not found")
        
        # unlink the child from the HA tree
        child.set_root(None)
        child._parent = None
        self._children.remove(child)

        return child

    def remove_all(self, children: list[Element]) -> Element:
        # remove each child
        for child in children:
            self.remove(child)
        
        # return the last removed child
        return children[-1]

    def pre_add(self, child: Element) -> Element:
        return child

    def post_add(self, child) -> Element:
        return child

