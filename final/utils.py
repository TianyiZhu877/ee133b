from scipy.stats import truncnorm

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


