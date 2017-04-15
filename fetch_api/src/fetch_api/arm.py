#! /usr/bin/env python

import trajectory_msgs.msg
import rospy
import control_msgs.msg
import actionlib

from .arm_joints import ArmJoints

TIME_FROM_START = 5  

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self._client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        trajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        trajPoint.positions = arm_joints.values()
        # TODO: Set time of trajectory point
        trajPoint.time_from_start = rospy.Duration(TIME_FROM_START)
        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint name to list
        goal.trajectory.joint_names = arm_joints.names()
        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points = [trajPoint]
        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()
