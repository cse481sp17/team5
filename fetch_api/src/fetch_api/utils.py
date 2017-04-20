import tf.transformations as tft
import math
# import numpy as np

def calc_minkowski_distance(a, b, p):
    """Calculate the minkowski distance between the 2 points"""
    aggreg = 0.0
    aggreg += pow(abs(a.x-b.x), p)
    aggreg += pow(abs(a.y-b.y), p)
    aggreg += pow(abs(a.z-b.z), p)
    return pow(aggreg, 1.0/p)

def quaternion_to_yaw(quaternion):
    """Get the yaw of the provided quaternion in radians"""
    # Yaw - Z axis

    # 1. Convert the quaternion into a rotation matrix
    matrix = tft.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    # 2. Get the roation of the X-axis
    # 2.1 Get the x  and value of the x-axis
    x = matrix[0, 0]
    y = matrix[1, 0]

    # 2.2 Get the arc-tangent of y and x in degrees
    theta_rads = math.atan2(y, x)
    #theta_degs = theta_rads * 180 / math.pi
    return theta_rads
    
def calculate_necessary_yaw(current_location, desired_location):
    """Calculate the necessary yaw to point towards the desired location in radians"""
    ang1 = quaternion_to_yaw(current_location)
    ang2 = quaternion_to_yaw(desired_location)
    if ang1 < 0 and ang2 > 0:
        ang1 += 2*math.pi
    result = (ang1 - ang2) 
    # if result > math.pi:
    #     result -= 2*math.pi
    # if result < -math.pi:
    #     result += 2*math.pi
    return result