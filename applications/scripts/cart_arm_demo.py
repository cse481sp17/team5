#! /usr/bin/env python

import fetch_api
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass



def main():
    # ... init ...
    rospy.init_node('cart_arm_demo')
    arm = fetch_api.Arm()

    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)
   
   # Move the arm
    wait_for_time()
    argv = rospy.myargv()

    torso = fetch_api.Torso()
    torso.set_height(fetch_api.Torso.MAX_HEIGHT)

    pose1 = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
    pose2 = Pose(Point(0.047, 0.545, 1.822), Quaternion(-0.274, -0.701, 0.173, 0.635))
    gripper_pose_stamped = PoseStamped()
    gripper_pose_stamped.header.frame_id = 'base_link'

    
    gripper_poses = [pose1, pose2]

    while not rospy.is_shutdown():
        for vals in gripper_poses:
            #  use your new arm method and check the error code
            gripper_pose_stamped.header.stamp = rospy.Time.now()
            gripper_pose_stamped.pose = vals

            error = arm.move_to_pose(gripper_pose_stamped)
            if error is not None:
                rospy.logerr(error)

if __name__ == '__main__':
    main()
