from hybrid_automaton import HybridAutomaton
from controller import *
from sensor import *
import behavioral
import numpy as np


if __name__ == '__main__':
    ha = HybridAutomaton('shove_grasp')

    home = HTransformController(name='home',
        reference_frame='base',
        goal=np.array([
            [0, 1,  0,  0.4],
            [1, 0,  0,  0],
            [0, 1,  -1, 0.4],
            [0, 0,  0,  1],
        ]),
        goal_is_relative=False,
        kp=np.array([[500, 500, 500, 100, 100, 100]]).T,
        kv=np.array([[30, 30, 30, 10, 10, 10]]).T,
        completion_times=np.array([[4]]),
        interpolation_type = 'cubic')
    shove = HTransformController(name='shove')

    control_mode = (ha
        .start(home)
        .when_time_elapsed(5.0)
        .then(GravityCompController(name='end')))

    print(ha.xml(indent=2))

    ha.run()
