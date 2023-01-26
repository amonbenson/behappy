from hybrid_automaton import HybridAutomaton
from controller import *
from sensor import *
from behavioral import *
from transform import Transform
import numpy as np


if __name__ == '__main__':
    ha = HybridAutomaton('shove_grasp')

    home_goal = (Transform()
        .translate(np.array([0.4, 0, 0.4]))
        .rotate([np.pi, 0, np.pi / 2]))

    home = HTransformController(name='home',
        reference_frame='base',
        goal=home_goal,
        goal_is_relative=False,
        kp=np.array([[500, 500, 500, 100, 100, 100]]).T,
        kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
        completion_times=np.array([[4]]),
        v_max=np.array([[]]),
        a_max=np.array([[]]),
        interpolation_type = 'cubic')
    shove = HTransformController(name='shove')

    control_mode = (ha
        .start(home)
        .when(time_elapsed(5.0))
        .end())

    print(ha.xml(indent=2))

    #ha.run()
