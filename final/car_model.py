import numpy as np
from node import Node

class carModel:
    def __init__(self, max_steer = np.pi/4, dt=0.01, mu=0.8, C_alpha=50000, F_z=4000, L=2.5, a=1.2, b=1.3,
                 a_min=-12, a_max=10,
                 controller_config = None):
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
        self.max_steer = max_steer
        self.a_min = a_min
        self.a_max = a_max

        self.controller_config = controller_config


    def max_steering_angle(self, node):
        """Calculates the maximum steering angle before tire slip occurs."""
        slip_limit = self.mu * self.F_z / self.C_alpha
        dynamic_max_steer = np.arctan(slip_limit / (1 + (self.a + self.b) / self.L * node.v_x))
        print('slip_limit', dynamic_max_steer)
        return min(dynamic_max_steer, self.max_steer)


    def generate_child(self, node, action, traj):
        """
        Propagates the state using the bicycle model and checks validity.
        
        Args:
            acceleration: Longitudinal acceleration input.
            steering_angle: Steering input.
        
        Returns:
            New Node if valid, otherwise None.
        """

        acceleration, steering_angle = action
        # Ensure steering angle and acceleration is within the allowed limit
        if (acceleration > self.a_max*1.01) or (acceleration < self.a_min*1.01):
            # print('acceleration', acceleration)
            return None
        
        max_steer = min(self.max_steering_angle(node), self.max_steer)
        if abs(steering_angle) > max_steer*1.01:
            # print('steering_angle', steering_angle, max_steer)
            return None  # Reject if steering causes slip
        
        # Update state using bicycle model equations
        new_v_x = node.v_x + acceleration * self.dt
        new_theta = node.theta + node.omega * self.dt

        # Approximate lateral velocity update
        new_v_y = node.v_y + (node.v_x * node.omega) * self.dt
        new_omega = (new_v_x / self.L) * np.tan(steering_angle)

        # Update position
        new_x = node.x + new_v_x * np.cos(new_theta) * self.dt
        new_y = node.y + new_v_x * np.sin(new_theta) * self.dt

        new_t = node.t + self.dt

        # Create new node
        new_node = Node(new_x, new_y, new_theta, new_v_x, new_v_y, new_omega, new_t, node, (acceleration, steering_angle))
                        # self.track_center, self.track_width, self.mu, self.C_alpha, self.F_z, self.L, self.a, self.b)
        print('new_node', new_node)
        # Reject if the new node is outside the track
        if not traj.is_inside_track((new_node.x, new_node.y)):
            return None

        return new_node

    def has_no_valid_children(self, node, traj):
        """Checks if this node has no valid child nodes (i.e., dead-end)."""
        # Try small steering and acceleration variations
        test_inputs = [
            (0, 0),  # No input
            (1, 0), (-1, 0),  # Acceleration
            (0, self.max_steering_angle(node)), (0, -self.max_steering_angle(node))  # Max steering
        ]
        for acc, steer in test_inputs:
            if self.generate_child(node, (acc, steer), traj) is not None:
                return False
        return True
    
    def controller(self, node, traj, idx, target_speed):
        d_lookahead = self.controller_config['k_lookahead'] * node.speed()
        dxy = traj.get_waypoint_bounded(idx + int(d_lookahead / traj.dstep)) - node.position()

        # Transform lookahead point into car's local coordinate system
        local_x = dxy[0] * np.cos(-node.theta) - dxy[1] * np.sin(-node.theta)
        local_y = dxy[0] * np.sin(-node.theta) + dxy[0] * np.cos(-node.theta)

        if local_x < 0:  
            print('Controller warning: Lookahead point should always be in front')

        # Compute curvature
        kappa = 2 * local_y / (local_x**2 + local_y**2)

        # Compute steering angle using bicycle model
        steer = np.arctan(self.L * kappa)

        # Compute acceleration
        v_error = target_speed - node.speed()
        acc = self.controller_config['P_acc'] * v_error

        return acc, steer
    


