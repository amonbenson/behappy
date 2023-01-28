import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *


ha = (HybridAutomaton('test')
    .start(JointPositionController('joint_pos', goal=None))
    .when('placeholder')
    .then(GravityCompController('grav_comp'))
    .when('placeholder')
    .finish())

print(ha.xml(indent=2))
