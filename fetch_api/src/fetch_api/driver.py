#! /usr/bin/env python

import rospy
import utils
import copy
import math


class Driver(object)
    def __init__(self, base):
         self.goal = None
         self._base = base

    def start():
        state = 'turn'
        goal = None
        while True:
            # Check if someone changed self.goal from outside
            if goal != self.goal:
                goal = copy.deepcopy(self.goal)
                # TODO: restart the turn/move sequence
                state = 'turn'
                desired_distance = calc_minkowski_distance(self._base._current_odom,goal,2) # Set this to how far the robot should move once pointed in the right direction

            if state == 'turn':
                # TODO: Compute how much we need to turn to face the goal
                if STILL_NEED_TO_TURN:
                    self._base.move(0, DIRECTION * ANGULAR_SPEED)
                else:
                    state = 'move'

            if state == 'move':
                # TODO: Compute how far we have moved and compare that to desired_distance
                # Make sure that the robot has the ability to drive backwards if it overshoots
                
                if STILL_NEED_TO_MOVE:
                    # TODO: possibly adjust speed to slow down when close to the goal
                    self._base.move(DIRECTION * SPEED, 0)

            rospy.sleep(0.1)