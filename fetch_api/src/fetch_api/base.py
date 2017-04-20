#! /usr/bin/env python

import rospy
import utils
import copy
import math
from geometry_msgs.msg import Twist
import nav_msgs.msg


class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_amath.pi.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # TODO: Create publisher
        self._basePub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, self.odom_callback)
        self._current_odom = None

    def odom_callback(self, msg):
        self._current_odom = msg

   

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
            distance: The distance, in meters, to move. A positive value
                means forward, negative means backward.
            speed: The speed to travel, in meters/second.
        """
        if(distance < 0):
            speed *= -1
            distance *= -1

        rate = rospy.Rate(10)
        while(self._current_odom is None):
            rate.sleep()
        start = copy.deepcopy(self._current_odom)
        curr_distance = utils.calc_minkowski_distance(start.pose.pose.position, self._current_odom.pose.pose.position, 2)
        while curr_distance < distance:
            # print curr_distance - distance
            self.move(speed, 0)
            curr_distance = utils.calc_minkowski_distance(start.pose.pose.position, self._current_odom.pose.pose.position, 2)
            rate.sleep()
        self.stop()

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """

        angular_distance = angular_distance % (2 * math.pi)
        if angular_distance > math.pi:
            speed *= -1
            angular_distance = 2* math.pi - angular_distance
        

        rate = rospy.Rate(10)
        while(self._current_odom is None):
            rate.sleep()
        start = copy.deepcopy(self._current_odom)

        curr_angular_distance = utils.calculate_necessary_yaw(self._current_odom.pose.pose.orientation, start.pose.pose.orientation)    

        while abs(curr_angular_distance) < angular_distance:
            # print angular_distance
            self.move(0, speed)
            curr_angular_distance = utils.calculate_necessary_yaw(self._current_odom.pose.pose.orientation, start.pose.pose.orientation)
            rate.sleep()

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
        message = Twist()
        message.linear.x = linear_speed
        message.angular.z = angular_speed
        self._basePub.publish(message)

    def stop(self):
        """Stops the mobile base from moving.
        """
        message = Twist()
        message.linear.x = 0
        message.angular.z = 0
        self._basePub.publish(message)
