from hybrid_automaton import HybridAutomaton
from jump_condition import JumpCriterion
from controller import *
from sensor import *
from transform import Transform
import numpy as np


if __name__ == '__main__':
    ha = HybridAutomaton('shove_grasp')

    ALIGN_ROTATION = 1.3
    SHOVE_DISTANCE = 0.5

    (ha
        .start(JointPositionController(name='home',
            goal=np.array([[-0.01, -0.15, -0.03, -2.14, -0.03, 1.99, -2.38]]).T,
            goal_is_relative=False,
            kp=np.array([[60, 60, 60, 60, 60, 20, 20]]).T,
            kv=np.array([[30, 30, 30, 20, 20, 20, 10]]).T,
            completion_times=np.array([[8.0]]),
            v_max=np.array([[]]),
            a_max=np.array([[]]),
            interpolation_type = 'cubic'))
        .when(time_elapsed(8.0))
        .then(HTransformController(name='descend_arm_until_contact',
            goal=Transform().translate([0, 0, 0.31]),
            goal_is_relative=False,
            reference_frame='EE',
            operational_frame='EE',
            kp=np.array([[500, 500, 500, 100, 100, 100]]).T,
            kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
            completion_times=np.array([[8.0]]),
            v_max=np.array([[]]).T,
            a_max=np.array([[]]),
            priority=0,
            interpolation_type = 'cubic'))
        .when(frontal_contact(-3.5))
        .then(HTransformController(name='retract',
            goal=Transform().translate([0, 0, -0.05]),
            goal_is_relative=False,
            reference_frame='EE',
            operational_frame='EE',
            kp=np.array([[500, 500, 500, 100, 100, 100]]).T,
            kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
            completion_times=np.array([[2.0]]),
            v_max=np.array([[]]).T,
            a_max=np.array([[]]),
            priority=0,
            interpolation_type = 'cubic'))
        .when(time_elapsed(2.0))
        .then(HTransformController(name='reorient',
            goal=(Transform()
                .rotate([0, -ALIGN_ROTATION, 0])
                .translate([0.1, 0, 0.03])), # move away from the object while rotating
            goal_is_relative=False,
            reference_frame='EE',
            operational_frame='object',
            kp=np.array([[500, 500, 500, 100, 100, 100]]).T,
            kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
            completion_times=np.array([[8.0]]),
            v_max=np.array([[]]).T,
            a_max=np.array([[]]),
            priority=0,
            interpolation_type = 'cubic'))
        .when(time_elapsed(8.0))
        .then(HTransformController(name='descend_hand_until_contact',
            goal=Transform().translate([0.1, 0, -0.02]),
            goal_is_relative=False,
            reference_frame='EE',
            operational_frame='EE',
            kp=np.array([[500, 500, 500, 100, 100, 100]]).T,
            kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
            completion_times=np.array([[5.0]]),
            v_max=np.array([[]]).T,
            a_max=np.array([[]]),
            priority=0,
            interpolation_type = 'cubic'))
        .when(back_of_hand_contact(-0.1))
        .then(HTransformController(name='move_along_ground',
            goal=(Transform()
                .translate([-SHOVE_DISTANCE * np.cos(ALIGN_ROTATION), 0, SHOVE_DISTANCE * np.sin(ALIGN_ROTATION)])),
            goal_is_relative=False,
            reference_frame='EE',
            operational_frame='EE',
            kp=np.array([[1000, 1000, 1000, 100, 100, 100]]).T,
            kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
            completion_times=np.array([[10.0]]),
            v_max=np.array([[]]).T,
            a_max=np.array([[]]),
            priority=0,
            interpolation_type = 'cubic'))
        .when(frontal_contact(2))
        .finish())

    print(ha.xml(indent=2))

    ha.run()
