from .hybrid_automaton import HybridAutomaton
from .controller import HTransformController


if __name__ == '__main__':
    ha = HybridAutomaton('shove_grasp')

    ha.start(HTransformController(name='home'))

    print(ha.xml(indent=2))
