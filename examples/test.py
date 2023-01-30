import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *


ha = (start_ha('test')
    .then(JointPositionController('home',
        goal=[-0.01, -0.15, -0.03, -2.14, -0.03, 1.99, -2.38],
        completion_times=8.0
    ))
    .then(HTransformController('descend_arm_until_contact',
        goal=Transform().translate([0, 0, 0.31]),
        reference_frame='EE',
        operational_frame='EE',
        completion_times=8.0
    ))
    .finish())

print(ha.xml(indent=2))

ha.run()
