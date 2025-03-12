import numpy as np
from node import Node

class carModel:
    def __init__(self, dt=0.01, mu=0.8, C_alpha=50000, F_z=4000, L=2.5, a=1.2, b=1.3):
        """
        Represents a state in the RRT tree with bicycle model dynamics.
        
        Args:
            x, y: Position of the car center.
            theta: Yaw angle (heading).
            v_x, v_y: Longitudinal and lateral velocities.
            omega: Yaw rate.
            dt: Time step for state propagation.
            track_center: 2D numpy array of track centerline points (shape: 2 * n).
            track_width: 1D numpy array of track widths (shape: n).
            mu, C_alpha, F_z, L, a, b: Vehicle parameters for slip calculation.
        """
        # self.x = x
        # self.y = y
        # self.theta = theta
        # self.v_x = v_x
        # self.v_y = v_y
        # self.omega = omega

        # self.track_center = track_center
        # self.track_width = track_width
        self.dt = dt
        self.mu = mu
        self.C_alpha = C_alpha
        self.F_z = F_z
        self.L = L
        self.a = a
        self.b = b

    def max_steering_angle(self, node):
        """Calculates the maximum steering angle before tire slip occurs."""
        slip_limit = self.mu * self.F_z / self.C_alpha
        return np.arctan(slip_limit / (1 + (self.a + self.b) / self.L * node.v_x))


    def generate_child(self, node, acceleration, steering_angle, traj):
        """
        Propagates the state using the bicycle model and checks validity.
        
        Args:
            acceleration: Longitudinal acceleration input.
            steering_angle: Steering input.
        
        Returns:
            New Node if valid, otherwise None.
        """
        # Ensure steering angle is within the allowed limit
        max_steer = self.max_steering_angle(node)
        if abs(steering_angle) > max_steer:
            return None  # Reject if steering causes slip
        
        # Update state using bicycle model equations
        new_v_x = node.v_x + acceleration * self.dt
        new_theta = node.theta + node.omega * self.dt

        # Approximate lateral velocity update
        new_v_y = node.v_y + (node.v_x * self.omega) * self.dt
        new_omega = (new_v_x / self.L) * np.tan(steering_angle)

        # Update position
        new_x = node.x + new_v_x * np.cos(new_theta) * self.dt
        new_y = node.y + new_v_x * np.sin(new_theta) * self.dt

        new_t = node.t + self.dt

        # Create new node
        new_node = Node(new_x, new_y, new_theta, new_v_x, new_v_y, new_omega, new_t, node, (acceleration, steering_angle))
                        # self.track_center, self.track_width, self.mu, self.C_alpha, self.F_z, self.L, self.a, self.b)
        
        # Reject if the new node is outside the track
        if not traj.is_inside_track((new_node.x, new_node.y)):
            return None

        return new_node

    def has_no_valid_children(self, node):
        """Checks if this node has no valid child nodes (i.e., dead-end)."""
        # Try small steering and acceleration variations
        test_inputs = [
            (0, 0),  # No input
            (1, 0), (-1, 0),  # Acceleration
            (0, self.max_steering_angle(node)), (0, -self.max_steering_angle(node))  # Max steering
        ]
        for acc, steer in test_inputs:
            if self.generate_child(acc, steer) is not None:
                return False
        return True
