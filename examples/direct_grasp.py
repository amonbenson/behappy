import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *
import rospy


ALIGN_ROTATION = 1.3
SHOVE_DISTANCE = 0.5


rospy.init_node('softhand_controller', anonymous=True)


ha = (start_ha('test')
    # move down until object contact
    .then(JointPositionController('home',
        goal=[-0.01, -0.15, -0.03, -2.14, -0.03, 1.99, -0.8],
        completion_times=2.0,
        interpolation_type='cubic'))
    .then(HTransformController('descend_arm_until_contact',
        goal=Transform().translate([0, 0, 0.31]),
        completion_times=5.0))
    .when(FrontalContact())

    # grasp the object
    .grasp('cage', pose=[0.0, 0.7, 0.5, 0.5, 0.5], time=1.0)
    .then(HTransformController('align',
        goal=(Transform()
            .rotate([0, 0.4, 0])
            .translate([-0.02, 0, 0])),
        operational_frame='object',
        completion_times=3.0,
        interpolation_type='cubic'))
    .grasp('grasp', pose=[1.0, 1.0, 0.8, 0.8, 1.0], time=1.0)
    .then(JointPositionController('pick_up',
        goal=[-0.01, -0.15, -0.03, -2.14, -0.03, 1.99, -0.8],
        completion_times=5.0,
        interpolation_type='cubic'))
    .finish())

print(ha.xml(indent=2))

ha.run()


rospy.spin()
