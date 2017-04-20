import tf.transformations as tft
import math
import numpy as np


def quaternion_to_yaw(quaternion):
    """Get the yaw of the provided quaternion in degrees"""
    # Yaw - Z axis

    # 1. Convert the quaternion into a rotation matrix
    matrix = tft.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    # 2. Get the roation of the X-axis
    # 2.1 Get the x  and value of the x-axis
    x = matrix[0, 0]
    y = matrix[1, 0]

    # 2.2 Get the arc-tangent of y and x in degrees
    theta_rads = np.atan2(y, x)
    theta_degs = theta_rads * 180 / math.pi

    return theta_degs
    
def calculate_necessary_yaw(current_location, desired_location):
    """Calculate the necessary yaw to point towards the desired location""""
    ang1 = quaternion_to_yaw(current_location)
    ang2 = quaternion_to_yaw(desired_location)
    return ang1 - ang2
