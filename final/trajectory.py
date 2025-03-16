import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
from scipy.ndimage import gaussian_filter
from scipy.spatial import KDTree
from utils import *

class Trajectory:
    def __init__(self, points, widths = 2, width_func = None, dstep=0.1, sigma = 30.0, use_center_as_estim = False):

        
        # Calculate the cumulative distance along the line
        distances = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0)**2, axis=1)))
        distances = np.insert(distances, 0, 0)  # Add the starting point
        
        # Create an interpolation function for x and y coordinates
        interp_x = interp1d(distances, points[:, 0], kind='linear')
        interp_y = interp1d(distances, points[:, 1], kind='linear')
        # Generate new distances for interpolation
        new_distances = np.arange(0, distances[-1], dstep)
        # Interpolate the points
        interpolated_x = interp_x(new_distances)
        interpolated_y = interp_y(new_distances)
        interpolated_points = np.column_stack((interpolated_x, interpolated_y))
        self.dstep = dstep

        if not np.isscalar(widths):
            interp_width = interp1d(distances, widths, kind='linear')
            self.widths = interp_width(new_distances)
        else:
            self.widths = np.ones(len(new_distances))*widths
        
        
        # Smooth the interpolated points using a Savitzky-Golay filter
        # smoothed_x = savgol_filter(interpolated_points[:, 0], window_length, polyorder)
        # smoothed_y = savgol_filter(interpolated_points[:, 1], window_length, polyorder)
        smoothed_x = gaussian_filter(interpolated_points[:, 0], sigma=sigma)
        smoothed_y = gaussian_filter(interpolated_points[:, 1], sigma=sigma)
        self.points = np.column_stack((smoothed_x, smoothed_y))
        # print(new_distances)
        self.distances = new_distances
        self.tree = KDTree(self.points)

        if width_func is not None:
            self.widths = width_func(self.distances)

        self.tangent = self.points[1:] - self.points[:-1]
        self.tangent = np.vstack((self.tangent, self.tangent[-1]))  # Repeat the last point
        self.tangent = self.tangent / np.linalg.norm(self.tangent, axis=1, keepdims=True)

        tan_x = self.tangent[:, :1]
        tan_y = self.tangent[:, 1:]

        self.right_boundary = self.points + np.hstack((tan_y, -tan_x))*self.widths[:, np.newaxis]
        self.left_boundary = self.points + np.hstack((-tan_y, tan_x))*self.widths[:, np.newaxis]

        self.estim_path = self.points
        if not use_center_as_estim:
            estim_path_x = gaussian_filter(self.points[:, 0], sigma=sigma*1.6)
            estim_path_y = gaussian_filter(self.points[:, 1], sigma=sigma*1.6)
            self.estim_path = np.column_stack((estim_path_x, estim_path_y))
        # might want to interpolate again here?

        self.estim_max_speed = None
        self.left_max_speed = None
        self.right_max_speed = None

        self.max_width = np.max(self.widths)


    def nearest_point(self, point):
        dist, idx = self.tree.query(point)
        return idx, dist
    
    def set_constant_max_speed(self, speed):
        
        self.estim_max_speed = np.ones(self.points.shape[0]) * speed
        self.left_max_speed = np.ones(self.points.shape[0]) * speed
        self.right_max_speed = np.ones(self.points.shape[0]) * speed

    def is_inside_track(self, point):
        """Checks if the car is within track bounds."""
        # Find the closest track center point
        # dists = np.linalg.norm(self.track_center.T - np.array([self.x, self.y]), axis=1)
        # track_half_width = self.track_width[idx] / 2
        idx, dist = self.nearest_point(point)

        return abs(dist) <= self.widths[idx]

    def get_field_by_policy(self, policy):
        if policy == ESTIM:
            return self.estim_path
        
        if policy == LEFT:
            return self.left_boundary
        
        if policy == RIGHT:
            return self.right_boundary

    def get_waypoint_bounded(self, idx, policy = ESTIM):
        """Returns the waypoint at the given index, bounded by the trajectory length."""
        points = self.get_field_by_policy(policy)
        idx = max(min(idx, len(points) - 1), 0)
        return points[idx], idx
    
    def get_max_speed_by_policy(self, policy):
        if policy == ESTIM:
            return self.estim_max_speed
        
        if policy == LEFT:
            return self.left_max_speed
        
        if policy == RIGHT:
            return self.right_max_speed
        
    def set_max_speed_by_policy(self, policy, idx, val):
        if policy == ESTIM:
            self.estim_max_speed[idx] = val
        
        if policy == LEFT:
            self.left_max_speed[idx] = val
        
        if policy == RIGHT:
            self.right_max_speed[idx] = val
    

def sin_width_func(x, T = 6, min_A = 1.6, max_A = 2):
    return min_A + (max_A - min_A)*np.sin(x/T)


def get_sharpe_angle_traj():
    # points = [(0, 0), (1, 2), (3, 3), (6, 5), (10, 10)]
    points = [(-10, 0), (30, 0), (30, 20), (40, 20), (50, 10), (80, 10)]
    # points = [(0, 0), (0,5), (5,5)]
    points = np.array(points)
    # traj = Trajectory(points)
    traj = Trajectory(points, width_func = sin_width_func)
    return traj


def get_sin_traj(T = 3, A = 5):
    x = np.arange(0, 50, 0.1)
    y = A*np.sin(x/T)
    points = np.column_stack((x, y))
    # traj = Trajectory(points)
    traj = Trajectory(points, width_func = sin_width_func)
    return traj



# if __name__ == '__main__':


