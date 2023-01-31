import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
from behappy.softhand import SofthandController

controller = SofthandController(debug=True)
controller.move_to([0.2, 0.2, 0.7, 0.7, 0.5], time=1.0)

time.sleep(1)

controller.reset()
