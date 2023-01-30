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
        completion_times=5.0))
    .when(FrontalContact(1.0))

    # rotate sideways
    .then(HTransformController('retract',
        goal=Transform().translate([0, 0, -0.02]),
        completion_times=0.5))
    .then(HTransformController('align',
        goal=(Transform()
            .rotate([0, -ALIGN_ROTATION, 0])
            .translate([0.1, 0, 0.03])),
        operational_frame='object',
        completion_times=6.0))
    .then(HTransformController('descend_hand_until_contact',
        goal=Transform().translate([0.1, 0, -0.02]),
        completion_times=6.0,
        interpolation_type='cubic'))
    .when(BackOfHandContact(-0.8))

    # shove the object
    .then(HTransformController('move_along_ground',
        goal=(Transform()
            .translate([-SHOVE_DISTANCE * np.cos(ALIGN_ROTATION), 0, SHOVE_DISTANCE * np.sin(ALIGN_ROTATION)])),
        completion_times=10.0))
    .when(FrontalContact(5.0))
    .finish())

print(ha.xml(indent=2))

ha.run()
