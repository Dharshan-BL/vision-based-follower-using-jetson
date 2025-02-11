#=====================================================================================================================
# Year: 2025
# Organization: Technical University of Eindhoven
# Author: Dr. Ömür Arslan
# Maintainer: D. Bashkaran Latha
#
# Description: This script contains control tools for the unicycle model.
# Created on: February 10, 2025
#=====================================================================================================================

import numpy as np

def unicycle_gradient_ctrl_2D(gradient, yaw, lin_gain=1.0, ang_gain=1.0):

    gradient = np.asarray(gradient)
    yaw = np.asarray(yaw)

    tangent_vector = np.column_stack((np.cos(yaw), np.sin(yaw)))
    normal_vector = np.column_stack((-np.sin(yaw), np.cos(yaw)))

    lin_vel = lin_gain*np.sum(gradient * tangent_vector, axis=1)
    ang_vel = ang_gain*np.arctan2(np.sum(gradient*normal_vector, axis=1), np.sum(gradient*tangent_vector, axis=1))

    return lin_vel, ang_vel

def scale_velocities(lin_vel, ang_vel, max_lin, max_ang):
    """
    Scale linear and angular velocities to maintain the curvature of the motion
    while not exceeding the maximum values.
    """
    # Calculate the current magnitudes and compare with maxima
    scale_lin = abs(lin_vel / max_lin) if lin_vel != 0 else 0
    scale_ang = abs(ang_vel / max_ang) if ang_vel != 0 else 0

    # Determine the maximum scale used
    scale = max(scale_lin, scale_ang)

    # If scaling is needed (scale > 1), reduce both velocities proportionally
    if scale > 1:
        lin_vel /= scale
        ang_vel /= scale

    return lin_vel, ang_vel
