import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *


ha = HybridAutomaton('factory_test')

# try adding a single control mode
cm = ControlModeFactory(controllers=[JointPositionController(name='single_first')])
ha.add(cm)

# try adding a control mode collection
cmc = ControlModeCollectionFactory()
cmc.add(ControlModeFactory(controllers=[JointPositionController(name='collection_first')]))
cmc.add(ControlModeFactory(controllers=[JointPositionController(name='collection_second')]))
ha.add(cmc)


print(ha.xml(indent=2))
