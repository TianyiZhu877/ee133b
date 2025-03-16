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

        self.nearest_idx = None
        self.nearest_dist = None   # left negative, right positive
    
    def distance(self, other):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def speed(self):
        return np.sqrt(self.v_x**2 + self.v_y**2)
    
    def position(self):
        return np.array([self.x, self.y])
    
    def get_idx_dist(self, traj):
        if self.nearest_idx is None:
            self.nearest_idx, self.nearest_dist = traj.nearest_point(self.position())
            self.nearest_idx = int(self.nearest_idx)
        return self.nearest_idx, self.nearest_dist

    def is_valid(self, traj):
        idx, dist = self.get_idx_dist(traj)
        return abs(dist) <= traj.widths[idx]
    
    def build_waypoints(self):
        current = self
        waypoints = []
        while current.parent is not None:
            waypoints.append(current)
            current = current.parent
        
        return waypoints
            



    # def __eq__(self, other):
    #     return self.distance(self.state, other.state) < self.config['reach_threshold']

    def __str__(self):
        return (f'Node: x={self.x}, y={self.y}, theta={self.theta}, v_x={self.v_x}, v_y={self.v_y}, omega={self.omega}, t={self.t}')
