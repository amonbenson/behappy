from hybrid_automaton import HybridAutomaton
from controller import *
from sensor import *
from behavioral import *
from transform import Transform
import numpy as np


if __name__ == '__main__':
    ha = HybridAutomaton('shove_grasp')

    home_goal = (Transform()
        .translate(np.array([0.5, 0, 0.4]))
        .rotate([np.pi, 0, np.pi / 4]))

    home = HTransformController(name='home',
        goal=home_goal,
        goal_is_relative=False,
        reference_frame='base',
        kp=np.array([[500, 500, 500, 100, 100, 100]]).T,
        kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
        completion_times=np.array([[4]]),
        v_max=np.array([[]]),
        a_max=np.array([[]]),
        priority=0,
        interpolation_type = 'cubic')
    descend = HTransformController(name='descend',
        goal=Transform().translate([0, 0, 0.2]),
        goal_is_relative=False,
        reference_frame='EE',
        operational_frame='base',
        kp=np.array([[500, 500, 500, 100, 100, 100]]).T,
        kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
        completion_times=np.array([[5]]),
        v_max=np.array([[]]).T,
        a_max=np.array([[]]),
        priority=0,
        interpolation_type = 'cubic')
    reorient = HTransformController(name='reorient',
        goal=Transform().rotate([0, -0.5, 0]),
        goal_is_relative=False,
        reference_frame='EE',
        operational_frame='base',
        kp=np.array([[500, 500, 500, 100, 100, 100]]).T,
        kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
        completion_times=np.array([[5]]),
        v_max=np.array([[]]).T,
        a_max=np.array([[]]),
        priority=0,
        interpolation_type = 'cubic')

    control_mode = (ha
        .start(home)
        .when(time_elapsed(4.0))
        .then(descend)
        .when(time_elapsed(5.0))
        .then(reorient)
        .when(time_elapsed(5.0))
        .finish())

    print(ha.xml(indent=2))

    ha.run()
