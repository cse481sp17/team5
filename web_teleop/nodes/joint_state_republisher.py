#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from joint_state_reader import JointStateReader


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('joint_state_republisher')
    wait_for_time()
    joint_list = ['torso_lift_joint', 'head_pan_joint', 'head_tilt_joint']
    torso_pub = rospy.Publisher('joint_state_republisher/torso_lift_joint',
                                Float64)
    head_pan_pub = rospy.Publisher('joint_state_republisher/head_pan_joint',
                                Float64)
    head_tilt_pub = rospy.Publisher('joint_state_republisher/head_tilt_joint',
                                Float64)
    reader = JointStateReader()
    rospy.sleep(0.5)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        value = reader.get_joints(joint_list)
        message = Float64()
        message.data = value[0]
        torso_pub.publish(message)
        message1 = Float64()
        message1.data = value[1]
        head_pan_pub.publish(message1)
        message2 = Float64()
        message2.data = value[2]
        head_tilt_pub.publish(message2)
        rate.sleep()


if __name__ == '__main__':
    main()