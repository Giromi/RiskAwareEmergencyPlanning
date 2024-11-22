from scipy.spatial.transform import Rotation as Rot
import numpy as np
import math
# Ref. pythonRobotic

def angle_mod(x, zero_2_2pi=False, degree=False) -> float or np.ndarray:
    """
    Angle modulo operation
    Default angle modulo range is [-pi, pi)

    Parameters
    ----------
    x : float or array_like
        A angle or an array of angles. This array is flattened for
        the calculation. When an angle is provided, a float angle is returned.
    zero_2_2pi : bool, optional
        Change angle modulo range to [0, 2pi)
        Default is False.
    degree : bool, optional
        If True, then the given angles are assumed to be in degrees.
        Default is False.

    Returns
    -------
    ret : float or ndarray
        an angle or an array of modulated angle.

    Examples
    --------
    >>> angle_mod(-4.0)
    2.28318531

    >>> angle_mod([-4.0])
    np.array(2.28318531)

    >>> angle_mod([-150.0, 190.0, 350], degree=True)
    array([-150., -170.,  -10.])

    >>> angle_mod(-60.0, zero_2_2pi=True, degree=True)
    array([300.])

    """
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle

def mod2pi(theta) -> float or np.ndarray:
    return angle_mod(theta, zero_2_2pi=True)

def msg(msg):
    print(msg)



def smooth_yaw(yaw):
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]
    return yaw


################################ About Rotation ################################

def rot_mat_to_2d(angle):
    """
    Create 2D rotation matrix from an angle

    Parameters
    ----------
    angle :

    Returns
    -------
    A 2D rotation matrix

    Examples
    --------
    >>> angle_mod(-4.0)
    """
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]

def webots_orientation_to_yaw(orientation: list) -> float:
    """
    Convert Webots orientation data (1D array of 9 elements) to yaw.

    Parameters
    ----------
    orientation : list or np.ndarray
        A 1D array of length 9 representing a 3x3 rotation matrix in row-major order.

    Returns
    -------
    float
        Yaw angle (rotation around the z-axis) in radians.

    Raises
    ------
    ValueError
        If the input orientation is not a valid 9-element array.
    """
    if len(orientation) != 9:
        raise ValueError("Orientation must be a list or array of 9 elements.")
    
    # Convert the 1D list to a 3x3 matrix
    rotation_matrix = np.array(orientation).reshape(3, 3)
    
    # Extract yaw angle from the rotation matrix
    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return yaw
