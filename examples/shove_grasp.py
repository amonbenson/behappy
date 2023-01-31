import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *
import rospy


ALIGN_ROTATION = 0.4
SHOVE_DISTANCE = 0.5


rospy.init_node('softhand_controller', anonymous=True)

home_goal = [-0.7029590606689453, 0.02395542897284031, 0.6823617219924927, -2.160890817642212, -0.06866637617349625, 2.1995151042938232, -2.359015941619873]


ha = (start_ha('test')
    # move down until object contact
    .then(JointPositionController('home',
        goal=home_goal,
        completion_times=3.0,
        interpolation_type='cubic'))
    .then(HTransformController('descend_arm_until_contact',
        goal=Transform().translate([0, 0, 0.31]),
        completion_times=5.0))
    .when(FrontalContact())

    # rotate sideways
    .grasp('cage', pose=[0.2, 0.2, 0.8, 0.4, 0.0], time=1.0)
    .then(HTransformController('align',
        goal=(Transform()
            .rotate([0, -ALIGN_ROTATION, 0])
            .translate([0.03, 0, -0.03])),
        operational_frame='object',
        completion_times=2.0))

    # shove the object
    .grasp('cage_drag', pose=[0.2, 0.2, 0.8, 0.4, 0.0], time=1.0)
    .then(HTransformController('move_along_ground',
        goal=(Transform()
            .translate([-SHOVE_DISTANCE * np.cos(ALIGN_ROTATION - 0.07), 0, SHOVE_DISTANCE * np.sin(ALIGN_ROTATION - 0.07)])),
        completion_times=10.0,
        interpolation_type='cubic'))
    .when(SidewaysContact())

    # grasp the object
    .grasp('uncage', pose=[0.5, 0.5, 0.0, 0.0, 0.0], time=1.0)
    .then(HTransformController('rotate_ee',
        goal=Transform()
            .translate([-0.03, 0.05, 0])
            .rotate([0.3, -0.6, -0.3]),
        operational_frame='object',
        completion_times=5.0))
    .grasp('grasp', pose=[1.0, 1.0, 0.8, 0.8, 1.0], time=1.0)
    .then(JointPositionController('pick_up',
        goal=home_goal,
        completion_times=8.0,
        interpolation_type='cubic'))
    .finish())

print(ha.xml(indent=2))

ha.run()
rospy.spin()
