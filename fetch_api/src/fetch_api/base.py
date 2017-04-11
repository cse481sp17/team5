#! /usr/bin/env python

# TODO: import ????????_msgs.msg
import rospy
from  geometry_msgs.msg import Twist


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
	self._basePub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # TODO: Create Twist msg
        # TODO: Fill out msg
        # TODO: Publish msg
	#        rospy.logerr('Not implemented.')
	message = Twist()
	message.linear.x = linear_speed
	message.angular.z = angular_speed
	self._basePub.publish(message)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # TODO: Publish 0 velocity
	message = Twist()
	message.linear.x = 0
	message.angular.z = 0
	self._basePub.publish(message)
        # rospy.logerr('Not implemented.')
