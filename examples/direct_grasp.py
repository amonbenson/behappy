import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *


ALIGN_ROTATION = 1.3
SHOVE_DISTANCE = 0.5


ha = (start_ha('test')
    # move down until object contact
    .then(JointPositionController('home',
        goal=[-0.01, -0.15, -0.03, -2.14, -0.03, 1.99, -2.38],
        completion_times=6.0,
        interpolation_type='cubic'))
    .then(HTransformController('descend_arm_until_contact',
        goal=Transform().translate([0, 0, 0.31]),
        completion_times=8.0))
    .when(FrontalContact(1.0))

    # grasp the object
    .finish())

print(ha.xml(indent=2))

ha.run()
