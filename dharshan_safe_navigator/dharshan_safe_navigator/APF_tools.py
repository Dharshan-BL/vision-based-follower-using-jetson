#=====================================================================================================================
# Year: 2025
# Organization: Technical University of Eindhoven
# Author: Dr. Ömür Arslan
# Maintainer: D. Bashkaran Latha
#
# Description: This script contains the tools for the Artificial Potential Field (APF) method for navigation.
# Created on: February 10, 2025
#=====================================================================================================================

import numpy as np
from scipy.spatial.distance import cdist    


# Math tools and functions
def distance_to_goal(position, goal=0):
    """
    Computes squared distance to goal
    """

    position = np.asarray(position)
    goal = np.asarray(goal)

    # Check if the input is a 1D array
    if position.ndim == 1:
        # Compute the squared norm of the entire 1D array
        distance = np.sum((position-goal)**2)
    
    # If the input is a 2D array, compute the squared norms of each row
    elif position.ndim == 2:
        distance =  np.sum((position-goal)**2, axis=1)
    
    else:
        raise ValueError("Input must be a 1D or 2D numpy array")
    
    return distance

def distance_to_obstacle(position, obstacle, tolerance=0.0):
    """
    Compute distance of a position to obstacle points at a given tolerance
    """
    position = np.array(position).reshape(1, 2)
    obstacle = np.array(obstacle).reshape(1, 2)

    distance = cdist(position, obstacle, metric='euclidean') - tolerance
    distance = np.maximum(distance, 0.0)
    distance = np.power(distance, 2)

    return distance

def sigmoid_threshold_tanh(x, rate=1.0):
    
    x = np.asarray(x)
    x = np.maximum(x, 0.0)
    y = np.tanh(rate * x)

    return y

def sigmoid_threshold_gradient_tanh(x, rate=1.0):
    """
    Applies a tanh-based sigmoid-like gradient transformation to the input array `x`.
    The transformation is clamped for non-positive values of `x` and scaled by a given rate.

    Parameters:
        x (array-like): Input array containing the elements to which the transformation is applied.
        rate (float): Scaling factor that controls the steepness of the gradient. Default is 1.0.

    Returns:
        np.ndarray: Transformed array where:
                     - For x > 0, elements are transformed by the gradient of the tanh function.
                     - For x <= 0, elements are set to 0.
    """
    
    # Convert input to a numpy array to enable elementwise operations
    x = np.asarray(x)
    
    # Clamp negative values of x to 0 (non-negative inputs only)
    x = np.maximum(x, 0.0)
    
    # Compute the gradient of the tanh function: y = rate * (1 - tanh(x)^2)
    # This is the derivative of the tanh function, scaled by the rate
    y = rate * (1 - np.power(np.tanh(rate*x), 2))
    
    # Set the result to 0 for non-positive values of x, otherwise use the computed gradient
    z = np.where(x <= 0, 0.0, y)
    
    return z

# Artificial Potential Field (APF) tools and functions

def navigation_potential_attractive(position, goal, strength=1.0):
    
    potential = np.power(distance_to_goal(position, goal), strength)

    return potential

def navigation_potential_repulsive(position, obstacle, tolerance=0.0, threshold_decay=1.0):
    
    threshold_func = lambda x: sigmoid_threshold_tanh(x, rate=threshold_decay)

    dist2obst = distance_to_obstacle(position, obstacle, tolerance)

    potential = 1.0/np.prod(threshold_func(dist2obst), axis=1)

    return potential

def gradient_navigation_potential_attractive(position, goal, strength=1.0):

    position = np.asarray(position)
    goal = np.asarray(goal)

    dist2goal = np.reshape(distance_to_goal(position, goal), (-1, 1))
    gradient = 2*strength*np.power(dist2goal, strength - 1.0)*(position-goal)

    gradient = np.reshape(gradient, position.shape)

    return gradient

def gradient_navigation_potential_repulsive(position, obstacle, tolerance=0.0, threshold_decay=1.0):
    
    position = np.asarray(position)
    obstacle = np.asarray(obstacle)
    gradient_shape = position.shape
    
    if position.ndim == 1:
        position = np.reshape(position, (1, -1))
    if obstacle.ndim == 1:
        obstacle = np.reshape(obstacle, (1,-1))
    
    number_of_position = position.shape[0] 
    number_of_obstacle = obstacle.shape[0]
    number_of_dimension = position.shape[1]

    dist2obst = distance_to_obstacle(position, obstacle, tolerance)

    threshold_func = lambda x: sigmoid_threshold_tanh(x, rate=threshold_decay)
    gradient_threshold_func = lambda x: sigmoid_threshold_gradient_tanh(x, rate=threshold_decay)

    gradient_scale = - gradient_threshold_func(dist2obst)/threshold_func(dist2obst)
    
    gradient_scale = np.tile(np.expand_dims(gradient_scale, axis=1), (1, number_of_dimension, 1))
    position = np.tile(np.expand_dims(position, axis=2), (1, 1, number_of_obstacle))
    obstacle = np.tile(np.expand_dims(obstacle.T, axis=0), (number_of_position, 1, 1))

    gradient = np.sum(gradient_scale*(position - obstacle), axis=2)
    gradient = gradient/np.reshape(np.prod(threshold_func(dist2obst), axis=1), (-1,1))

    gradient = np.reshape(gradient, gradient_shape) 

    return gradient


def gradient_navigation_potential(position, goal, obstacle, attractive_strength=1.0, repulsive_tolerance=0.0, repulsive_threshold_decay=1.0):

    attractive_potential = navigation_potential_attractive(position, goal, strength=attractive_strength)
    attractive_potential = np.reshape(attractive_potential, (-1, 1))
    attractive_gradient = gradient_navigation_potential_attractive(position, goal, strength=attractive_strength)

    repulsive_potential = navigation_potential_repulsive(
        position, 
        obstacle,
        tolerance=repulsive_tolerance, 
        threshold_decay=repulsive_threshold_decay
        )
    repulsive_potential = np.reshape(repulsive_potential, (-1, 1))
    repulsive_gradient = gradient_navigation_potential_repulsive(position, obstacle,
                            tolerance=repulsive_tolerance, threshold_decay=repulsive_threshold_decay)

    gradient = attractive_gradient*repulsive_potential+attractive_potential*repulsive_gradient
    
    return gradient
