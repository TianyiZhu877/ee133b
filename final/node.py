import numpy as np
from typing import Optional

class Node:
    def __init__(self, x, y, theta, v_x, v_y, omega, t, parent:Optional[Node]=None, action=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.v_x = v_x
        self.v_y = v_y
        self.omega = omega
        self.t = t
        self.parent = parent
        self.last_action = action


