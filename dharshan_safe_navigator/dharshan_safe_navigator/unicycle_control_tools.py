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

def scale_velocities(lin_vel, ang_vel, max_lin_vel, max_ang_vel):
    """
    Scale linear and angular velocities to maintain the curvature of the motion
    while not exceeding the maximum values.
    """    
    # cap velocity to maintain curvature
    if lin_vel != 0.0 and ang_vel != 0.0:
        scaling_factor = min(max_lin_vel / abs(lin_vel), max_ang_vel / abs(ang_vel), 1)
    elif ang_vel != 0.0:
        scaling_factor = max_ang_vel / abs(ang_vel)

    elif lin_vel != 0.0:
        scaling_factor = max_lin_vel / abs(lin_vel)

    else:
        scaling_factor = 1.0
    

    lin_vel = lin_vel * scaling_factor
    ang_vel = ang_vel * scaling_factor

    if (max_lin_vel*abs(ang_vel) + max_ang_vel*abs(lin_vel)) >= (max_lin_vel*max_ang_vel):
        ctrl_scale = max_lin_vel*max_ang_vel/(max_lin_vel*abs(ang_vel) + max_ang_vel*abs(lin_vel))
    else:
        ctrl_scale = 1.0

    lin_vel = lin_vel * ctrl_scale
    ang_vel = ang_vel * ctrl_scale

    return lin_vel, ang_vel


def fwd_headway_unicycle_twist(robot_pose, goal_pose, headway_pose, head_gain=0.2):
        """
        """
        angle = robot_pose[2]
        x = np.array([robot_pose[0], robot_pose[1]])
        x_star = np.array([goal_pose[0], goal_pose[1]])
        x_epsilon = np.array([headway_pose[0], headway_pose[1]])

        ##### ADAPTIVE HEADWAY CONTROL #####
            
        goal_dist_norm = np.linalg.norm(x_star - x)
        # k_epsilon = np.linalg.norm(x_epsilon - x) / goal_dist_norm
        
        k_epsilon = 0.05

        unit_delta_dist = (x_star - x) / goal_dist_norm
        lin_vel_comp = np.dot(np.array([np.cos(angle), np.sin(angle)]), unit_delta_dist)
        lin_vel = (head_gain * goal_dist_norm * (lin_vel_comp - k_epsilon))/(1-k_epsilon*lin_vel_comp)
        
        ang_vel_comp = np.dot(np.array([-np.sin(angle), np.cos(angle)]), unit_delta_dist)
        ang_vel = head_gain / k_epsilon * ang_vel_comp
        
        return lin_vel, ang_vel


def basic_unicycle_twist(robot_pose, goal_pose, k_lin=1.0, k_ang=1.0):

        heading_angle = robot_pose[2]
        goal_angle = np.arctan2(goal_pose[1] - robot_pose[1], goal_pose[0] - robot_pose[0])
        angle_diff = np.arctan2(np.sin(goal_angle - heading_angle), np.cos(goal_angle - heading_angle))

        if abs(angle_diff) > 0.01:
            lin_vel = 0.0
            ang_vel = k_ang * angle_diff
        else:
            lin_vel = 2 * k_lin * np.sqrt((goal_pose[0] - robot_pose[0])**2 + (goal_pose[1] - robot_pose[1])**2)
            ang_vel = 0.0

        return lin_vel, ang_vel