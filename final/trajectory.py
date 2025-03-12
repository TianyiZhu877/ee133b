import numpy as np
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


def interpolate_and_smooth_line(points, dstep=1.0, sigma = 15.0): # , window_length=5, polyorder=3)
    """
    Interpolates a line represented by a list of 2D points to a higher resolution,
    ensuring that the separation between points is no larger than `dstep`.
    Then, smooths the line to ensure a slower change in tangent.

    Parameters:
    - points: List of 2D points as tuples or lists, e.g., [(x1, y1), (x2, y2), ...]
    - dstep: Maximum allowed separation between points after interpolation.
    - window_length: The length of the filter window for smoothing (must be odd).
    - polyorder: The order of the polynomial used for smoothing.

    Returns:
    - smoothed_points: List of smoothed 2D points as numpy arrays.
    """
    
    # Convert the list of points to a numpy array
    # points = np.array(points)
    
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
    
    # Smooth the interpolated points using a Savitzky-Golay filter
    # smoothed_x = savgol_filter(interpolated_points[:, 0], window_length, polyorder)
    # smoothed_y = savgol_filter(interpolated_points[:, 1], window_length, polyorder)
    smoothed_x = gaussian_filter(interpolated_points[:, 0], sigma=sigma)
    smoothed_y = gaussian_filter(interpolated_points[:, 1], sigma=sigma)
    smoothed_points = np.column_stack((smoothed_x, smoothed_y))
    
    return smoothed_points


if __name__ == '__main__':
    # Example usage:
    # points = [(0, 0), (1, 2), (3, 3), (6, 5), (10, 10)]
    points = [(-10, 0), (30, 0), (30, 20), (40, 20), (50, 10), (80, 10)]
    # points = [(0, 0), (0,5), (5,5)]
    points = np.array(points)

    x = np.arange(0, 50, 0.1)
    y = 5*np.sin(0.3*x)
    points = np.column_stack((x, y))

    smoothed_line = interpolate_and_smooth_line(points, dstep=0.1) #, window_length=30, polyorder=20)
    # print(smoothed_line)

    print(len(smoothed_line))
    plt.plot(points[:, 0], points[:, 1], 'bo-', label='Original Line', alpha=0.5)
    # plt.show()

    # Plot the smoothed line
    plt.plot(smoothed_line[:, 0], smoothed_line[:, 1], 'r-', label='Smoothed Line', linewidth=2)
    plt.axis('equal')
    plt.show()


