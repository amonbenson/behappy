import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *


ha = (HybridAutomaton()
    .start(JointPositionController(name='hello', goal=None))
    .finish())

print(ha.xml(indent=2))
