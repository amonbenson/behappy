import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *


ha = (hybrid_automaton('test')
    .start(JointPositionController('joint_pos', goal=None))
    .when(TimeElapsed(3.0))
    .then(GravityCompController('grav_comp'))
    .when(TimeElapsed(3.0))
    .finish())

print(ha.xml(indent=2))
