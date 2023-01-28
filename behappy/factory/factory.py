from dataclasses import dataclass
from abc import ABC, abstractmethod
from ..ha.element import Element


@dataclass
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
        for child in self._children:
            child.produce(dst)
