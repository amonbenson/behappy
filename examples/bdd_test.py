import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *


ha = hybrid_automaton('bdd_test')

(ha.start(JointPositionController(name='home'))
    .when(JumpConditionFactory(sensor=ClockSensor(), jump_condition=JumpCondition(goal=None)))
    .then(JointPositionController(name='home')))

print(ha.xml(indent=2))
