import numpy as np

def get_wheels_speed(encoderValues, oldEncoderValues, pulses_per_turn, delta_t):
    """
    Computes speed of the wheels based on encoder ticks
    Note: webots position sensor returns radians not ticks
    """
    # Calculate the change in angular position of the wheels:
    ang_diff_l = 2*np.pi*(encoderValues[0] - oldEncoderValues[0])/pulses_per_turn
    ang_diff_r = 2*np.pi*(encoderValues[1] - oldEncoderValues[1])/pulses_per_turn

    # Calculate the angular speeds:
    wl = ang_diff_l/delta_t
    wr = ang_diff_r/delta_t

    return wl, wr

def get_robot_speeds(wl, wr, r, l):
    """
    Computes robot linear and angular speeds
      r - wheel radius (m)
      l - wheel base (m) axle length
    """
    u = r/2.0 * (wr + wl)
    w = r/l * (wr - wl)
    
    return u, w

def get_robot_pose(u, w, x_old, y_old, theta_old, delta_t):
    """
    Updates robot pose based on heading and linear and angular speeds
    Linear approximation  that over small time step
    """
    delta_theta = w * delta_t
    theta = theta_old + delta_theta
    
    # wrap the angle to within standard range
    # [0, 2 pi] -> [-pi, pi]
    if theta >= np.pi:
        theta = theta - 2 * np.pi
    elif theta < -np.pi:
        theta = theta + 2 * np.pi

    delta_x = u * np.cos(theta) * delta_t
    delta_y = u * np.sin(theta) * delta_t
    x = x_old + delta_x
    y = y_old + delta_y
    
    return x, y, theta

