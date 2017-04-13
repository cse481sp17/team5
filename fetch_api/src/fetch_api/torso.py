#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg 
import trajectory_msgs.msg
import rospy
import control_msgs.msg
import actionlib

# TODO: ACTION_NAME = ???
JOINT_NAME = 'torso_lift_joint'
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self._client = actionlib.SimpleActionClient('torso_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if height > Torso.MAX_HEIGHT or height < Torso.MIN_HEIGHT:
            return
    
        # TODO: Create a trajectory point
        trajPoint = trajectory_msgs.msg.JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        trajPoint.positions = [height]
        # TODO: Set time of trajectory point
        trajPoint.time_from_start = rospy.Duration(TIME_FROM_START)
        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint name to list
        goal.trajectory.joint_names = [JOINT_NAME]
        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points = [trajPoint]
        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result()
       # rospy.logerr('Not implemented.')
