import numpy as np
from typing import Optional

class Node:
    def __init__(self, x, y, theta, v_x, v_y, omega, t, parent=None, action=None, 
                 config:dict = {'reach_threshold': 0.1, '#grid_blocks': 30}):
        self.x = x
        self.y = y
        self.theta = theta
        self.v_x = v_x
        self.v_y = v_y
        self.omega = omega
        self.t = t
        self.parent = parent
        self.last_action = action
        self.config = config
        self.action_steer_grid = np.zeros((self.config['#grid_blocks']))
    
    def distance(self, other):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def speed(self):
        return np.sqrt(self.v_x**2 + self.v_y**2)
    
    def position(self):
        return np.array([self.x, self.y])
    
    # def __eq__(self, other):
    #     return self.distance(self.state, other.state) < self.config['reach_threshold']

    def __str__(self):
        return (f'Node: x={self.x}, y={self.y}, theta={self.theta}, v_x={self.v_x}, v_y={self.v_y}, omega={self.omega}, t={self.t}')
