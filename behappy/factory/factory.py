from dataclasses import dataclass
from ..element import Element


@dataclass
class Factory(Element):
    PRIORITY = 0

    def pre_xml(self):
        # produce as the last step before converting to xml
        self.produce()

        # remove the factory itself from the element tree
        self._parent.remove(self)

    def produce(self, dst: Element):
        self._children = self._children or []

        # produce all children onto the dst
        for child in self._children:
            child.produce(dst)
