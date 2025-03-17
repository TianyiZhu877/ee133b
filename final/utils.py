from scipy.stats import truncnorm
import numpy as np

ESTIM = 0
LEFT = 1
RIGHT = 2



def tuncated_normal(low, high, mu, sigma, rand):
    
    # Convert to standard normal bounds
    a, b = (low - mu) / sigma, (high - mu) / sigma
    generated = int(truncnorm.rvs(a, b, loc=mu, scale=sigma, size=1, random_state = rand))
    print(low, high, mu, sigma, generated)
    # Generate a single sample
    return generated

def R(theta):
    return np.array([[np.cos(theta), -np.sin(theta)], 
                    [np.sin(theta),  np.cos(theta)]]).T


def angle_between_vectors(a, b, normal=None):
    """ Computes the signed angle between two 3D vectors.
    
    If a normal vector is provided, it determines the sign using the right-hand rule.
    """
    # Compute dot and cross product
    dot_product = np.dot(a, b)
    cross_product = np.cross(a, b)
    
    # Compute magnitudes
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    
    # Compute angle in radians using atan2 for sign preservation
    theta_rad = np.arctan2(np.linalg.norm(cross_product), dot_product)

    return theta_rad

