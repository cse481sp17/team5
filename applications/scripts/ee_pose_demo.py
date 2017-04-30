#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('ee_pose_demo')

    listener = tf.TransformListener()
    rospy.sleep(0.1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rospy.loginfo('Gripper:')
        rospy.loginfo(trans)
        rospy.loginfo(rot)
        rate.sleep()