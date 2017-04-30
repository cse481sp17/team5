#! /usr/bin/env python

import math
import numpy as np
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
import tf.transformations as tft

def transform_to_pose(matrix):
    pose = Pose()
    quat_from_mat = tft.quaternion_from_matrix(matrix)
    # print quat_from_mat
    pose.orientation = Quaternion(quat_from_mat[0], quat_from_mat[1], quat_from_mat[2], quat_from_mat[3])
    vector = np.dot(matrix, np.array([0, 0, 0, 1]))
    # print vector
    pose.position = Point(vector[0], vector[1], vector[2])
    return pose

base_link_mat = tft.quaternion_matrix([0, 0, 0.38268343, 0.92387953])
base_link_mat[0][3] = 0.6
base_link_mat[1][3] = -0.1
base_link_mat[2][3] = 0.7

object_pregrasp = tft.identity_matrix()
object_pregrasp[0][3] = -0.1

dot = np.dot(base_link_mat, object_pregrasp)

print transform_to_pose(dot)

