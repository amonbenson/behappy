import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *


ha = (hybrid_automaton('test')
    .start(JointPositionController('joint_pos', goal=None, completion_times=8.0))
    .when(GoalReached())
    .then(GravityCompController('grav_comp'))
    .when(GoalReached())
    .finish())

print(ha.xml(indent=2))
