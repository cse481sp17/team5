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
    joint_list = ['torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'upperarm_roll_joint', 'elbow_flex_joint', 'forearm_roll_joint', 'wrist_flex_joint', 'wrist_roll_joint']
    torso_pub = rospy.Publisher('joint_state_republisher/torso_lift_joint', Float64)
    head_pan_pub = rospy.Publisher('joint_state_republisher/head_pan_joint', Float64)
    head_tilt_pub = rospy.Publisher('joint_state_republisher/head_tilt_joint', Float64)
    shoulder_pan_pub = rospy.Publisher('joint_state_republisher/shoulder_pan_joint', Float64)
    shoulder_lift_pub = rospy.Publisher('joint_state_republisher/shoulder_lift_joint', Float64)
    upperarm_roll_pub = rospy.Publisher('joint_state_republisher/upperarm_roll_joint', Float64)
    elbow_flex_pub = rospy.Publisher('joint_state_republisher/elbow_flex_joint', Float64)
    forearm_roll_pub = rospy.Publisher('joint_state_republisher/forearm_roll_joint', Float64)
    wrist_flex_pub = rospy.Publisher('joint_state_republisher/wrist_flex_joint', Float64)
    wrist_roll_pub = rospy.Publisher('joint_state_republisher/wrist_roll_joint', Float64)
                                
    reader = JointStateReader()
    rospy.sleep(1)

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

        message3 = Float64()
        message3.data = value[3]
        shoulder_pan_pub.publish(message3)

        message4 = Float64()
        message4.data = value[4]
        shoulder_lift_pub.publish(message4)

        message5 = Float64()
        message5.data = value[5]
        upperarm_roll_pub.publish(message5)

        message6 = Float64()
        message6.data = value[6]
        elbow_flex_pub.publish(message6)

        message7 = Float64()
        message7.data = value[7]
        forearm_roll_pub.publish(message7)

        message8 = Float64()
        message8.data = value[8]
        wrist_flex_pub.publish(message8)

        message9 = Float64()
        message9.data = value[9]
        wrist_roll_pub.publish(message9)

        rate.sleep()


if __name__ == '__main__':
    main()