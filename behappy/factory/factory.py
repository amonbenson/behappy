from dataclasses import dataclass
from abc import ABC, abstractmethod
from ..ha.element import Element


class Factory(Element, ABC):
    ALLOWED_CHILDREN = []
    PRIORITY = 0

    @abstractmethod
    def produce(self, dst: Element):
        raise NotImplementedError()


class FactoryCollection(Factory):
    ALLOWED_CHILDREN = [Factory]

    def produce(self, dst: Element):
        # produce all children onto the dst
        for child in self._children or []:
            child.produce(dst)

    @classmethod
    def from_item(cls, *kargs, children: Element = None, **kwargs) -> Element:
        # return a new collection instance with the added item
        collection = cls()
        return collection.add_item(*kargs, children=children, **kwargs)
    
    def add_item(self, *kargs, children: Element = None, **kwargs) -> Element:
        # instantiate a new item
        item_cls = self.ALLOWED_CHILDREN[0]
        item = item_cls(*kargs, **kwargs).add_all(children or [])

        # return the collection with the added item
        return self.add(item)

    def item_names(self):
        return [item.name for item in self._children]
