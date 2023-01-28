import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *


ha = HybridAutomaton('factory_test')

mode_0 = ControlModeFactory().add(JointPositionController(name='mode_0'))
mode_1 = ControlModeFactory().add(JointPositionController(name='mode_1'))
mode_2 = ControlModeFactory().add(JointPositionController(name='mode_2'))

# try adding a single control mode
ha.add(mode_0)

# try adding a control mode collection
cmc = ControlModeCollectionFactory()
cmc.add(mode_1)
cmc.add(mode_2)
ha.add(cmc)

# try adding a single control switch
jc1 = JumpConditionFactory(ClockSensor(), JumpCondition(goal=None, norm_weights=None))
cs = ControlSwitchFactory(
    sources=ControlModeCollectionFactory().add(mode_0).add(mode_2),
    targets=ControlModeCollectionFactory().add(mode_1).add(mode_2)).add(jc1)
ha.add(cs)


print(ha.xml(indent=2))
