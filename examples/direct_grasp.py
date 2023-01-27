import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from behappy import *
import numpy as np


if __name__ == '__main__':
    ha = HybridAutomaton('direct_grasp')

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
        .finish())

    print(ha.xml(indent=2))
    ha.run()
