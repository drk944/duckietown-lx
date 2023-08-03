from typing import Tuple

import numpy as np


def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
    """
        Args:
            ticks: Current tick count from the encoders.
            prev_ticks: Previous tick count from the encoders.
            resolution: Number of ticks per full wheel rotation returned by the encoder.
        Return:
            rotation_wheel: Rotation of the wheel in radians.
            ticks: current number of ticks.
    """

    delta_ticks = ticks-prev_ticks

    # Assuming no wheel slipping
    dphi = 2*np.pi*delta_ticks/resolution


    return dphi, ticks


def pose_estimation(
        R: float,
        baseline: float,
        x_prev: float,
        y_prev: float,
        theta_prev: float,
        delta_phi_left: float,
        delta_phi_right: float,
) -> Tuple[float, float, float]:
    """
    Calculate the current Duckiebot pose using the dead-reckoning approach.

    Args:
        R:                  radius of wheel (assumed identical) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x:                  estimated x coordinate
        y:                  estimated y coordinate
        theta:              estimated heading
    """
    
    d_l_k = R*delta_phi_left # delta l/r at timestep k = delta_phi * R
    d_r_k = R*delta_phi_right

    d_A_k = (d_l_k+d_r_k)/2     # Translation diff = delta_l + delta_r / 2
    d_T_k = (d_r_k-d_l_k)/(baseline) # Rotation diff = delta_l + delta_r /2 * baseline


    d_x_k = d_A_k * np.cos(theta_prev) # extract delta x and delta y in world coordinates
    d_y_k = d_A_k * np.sin(theta_prev)

    x_curr_1 = x_prev + d_x_k # Add to the update
    y_curr_1 = y_prev + d_y_k
    theta_curr_1 = theta_prev + d_T_k

    
    # w = [R, 2*R / baseline, 1]

    # x = np.array([[(delta_phi_left + delta_phi_right) * np.cos(theta_prev) / 2, (delta_phi_left + delta_phi_right) * np.sin(theta_prev) / 2, 0],         
                #   [0, 0, (delta_phi_right - delta_phi_left) / 2],
                #   [x_prev, y_prev, theta_prev]])

    # x_curr, y_curr, theta_curr = np.array(w).dot(x)

    return x_curr_1, y_curr_1, theta_curr_1
    # return x_curr, y_curr, theta_curr
    