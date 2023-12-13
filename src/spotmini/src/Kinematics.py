#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Imu
from transforms3d.euler import euler2mat
from spot_micro_kinematics.spot_micro_stick_figure import SpotMicroStickFigure

'''
    Orientation:
    X
    ^
    |
    |
    |
    --------> Z


    Y comes out of the screen towards you
'''

def leg_explicit_inversSe_kinematics(r_body_foot, leg_index, config):
    """Find the joint angles corresponding to the given body-relative foot position for a given leg and configuration
    
    Parameters
    ----------
    r_body_foot : numpy array (3,4)
        Matrix of the body-frame foot positions.
    leg_index : int
        Integer determining which leg to focus on.
    config : Config object
        Object of robot configuration parameters.
    
    Returns
    -------
    numpy array (3)
        Array of corresponding joint angles.

    Obsolete if using SpotMicro library, all angles found at once in 
    four_legs_inverse_kinematics()

    """
    (x, y, z) = r_body_foot

    # Distance from the leg origin to the foot, projected into the y-z plane
    R_body_foot_yz = (y ** 2 + z ** 2) ** 0.5

    # Distance from the leg's forward/back point of rotation to the foot
    R_hip_foot_yz = (R_body_foot_yz ** 2 - config.ABDUCTION_OFFSET ** 2) ** 0.5

    # Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
    # For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
    arccos_argument = config.ABDUCTION_OFFSETS[leg_index] / R_body_foot_yz
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    phi = np.arccos(arccos_argument)

    # Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis
    hip_foot_angle = np.arctan2(z, y)

    # Ab/adduction angle, relative to the positive y-axis
    abduction_angle = phi + hip_foot_angle

    # theta: Angle between the tilted negative z-axis and the hip-to-foot vector
    theta = np.arctan2(-x, R_hip_foot_yz)

    # Distance between the hip and foot
    R_hip_foot = (R_hip_foot_yz ** 2 + x ** 2) ** 0.5

    # Angle between the line going from hip to foot and the link L1
    arccos_argument = (config.LEG_L1 ** 2 + R_hip_foot ** 2 - config.LEG_L2 ** 2) / (
        2 * config.LEG_L1 * R_hip_foot
    )
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    trident = np.arccos(arccos_argument)

    # Angle of the first link relative to the tilted negative z axis
    hip_angle = theta + trident

    # Angle between the leg links L1 and L2
    arccos_argument = (config.LEG_L1 ** 2 + config.LEG_L2 ** 2 - R_hip_foot ** 2) / (
        2 * config.LEG_L1 * config.LEG_L2
    )
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    beta = np.arccos(arccos_argument)

    # Angle of the second link relative to the tilted negative z axis
    knee_angle = hip_angle - (np.pi - beta)

    return np.array([abduction_angle, hip_angle, knee_angle])

def callback(message):
    return message.data

def four_legs_inverse_kinematics(r_body_foot, config):
    """Find the joint angles for all twelve DOF correspoinding to the given matrix of body-relative foot positions.
    
    Parameters
    ----------
    r_body_foot : numpy array (3,4)
        Matrix of the body-frame foot positions. Each column corresponds to a separate foot.
    config : Config object
        Object of robot configuration parameters.
    
    Returns
    -------
    numpy array (3,4)
        Matrix of corresponding joint angles.

    I noticed our robot has 4 measurements (l1 - l4), but I hope l1
    can be negligible because of how small it is, l2-l4 correspond 
    to l1-l3 below accordingly
    """
    # all distances assumed to be meters and all angles radians

    sm = SpotMicroStickFigure()

    sm.set_absolute_foot_coordinates(r_body_foot)

    leg_angs = sm.get_leg_angles()

    alpha = leg_angs #np.zeros((3, 4))
    # for i in range(4):
    #     body_offset = config.LEG_ORIGINS[:, i]
    #     alpha[:, i] = leg_explicit_inverse_kinematics(
    #         r_body_foot[:, i] - body_offset, i, config
    #     )
    return alpha
