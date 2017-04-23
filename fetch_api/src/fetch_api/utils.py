import tf.transformations as tft
import math
import numpy as np
import numpy.linalg as la

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

def point_to_tuple(p1):
    return (p1.x, p1.y)

def calc_angl(p1, p2):
    p1 = point_to_tuple(p1)
    p2 = point_to_tuple(p2)
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return (ang1 - ang2)
    
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

def unit_vector(loc_vector):     
    """Calculate the unit vector of the location"""     
    return loc_vector / np.linalg.norm(loc_vector)  

def delta_ang(u, v):     
    """Calculate the delta andle in radians between location vectors v and u"""
    v_u = unit_vector(v)     
    u_u = unit_vector(u)     
    return np.arccos(np.clip(np.dot(v_u, u_u), -1.0, 1.0))

def point_to_array(p):
    return [p.x, p.y, p.z]


def calc_target_yaw(current_position, desired_position):
    return delta_ang(point_to_array(current_position), point_to_array       (desired_position))

def angular_diff(current, target):
    v = point_to_array(current)
    u = point_to_array(target)
    result = math.atan((u[1] - v[1]) / (u[0] - v[0]))
    if u[0] < v[0]:
        return result + math.pi
    return result