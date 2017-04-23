#! /usr/bin/env python

import rospy
import utils
import copy
import math
import time


class Driver(object):
    def __init__(self, base):
         self.goal = None
         self._base = base

    def start(self):

        state = None
        goal = None
        n_t_t = None
        check_angle = 0
        while True:
            # Check if someone changed self.goal from outside
            if goal != self.goal:
                goal = copy.deepcopy(self.goal)
                state = 'turn'
                speed = 0.2                
                start = copy.deepcopy(self._base._current_odom)                
                desired_distance = utils.calc_minkowski_distance(start.pose.pose.position, goal.position, 2)

                n_t_t = utils.angular_diff(self._base._current_odom.pose.pose.position, goal.position)
                print 'Need_to_turn: {}'.format(n_t_t)
                """
                if utils.calculate_necessary_yaw(self._base._current_odom.pose.pose.orientation, goal.orientation) < 0:
                    n_t_t *= -1
                """

                print 'Need_to_turn: {}'.format(n_t_t)
                '''print 'REQ_DIST: {}'.format(desired_distance)
                print 'goal.oriant'
                print goal.orientation.w
                print 'current oriant'
                print self._base._current_odom.pose.pose.orientation.w
                '''
                
            if state == 'turn':
                # TODO: Compute how much we need to turn to face the goal
                """
                required_angular_distance = utils.calculate_necessary_yaw(self._base._current_odom.pose.pose.orientation, goal.orientation)
                """
                """

                current_pos = self._base._current_odom.pose.pose.position
                current_yaw = utils.quaternion_to_yaw(self._base._current_odom.pose.pose.orientation)

                target_orientation = utils.calc_angl(current_pos, goal.position)
                print "Target Orientation: {}".format(target_orientation / 3.1415)
                print "Orientation Diff: {}".format(target_orientation - current_yaw)

                """

                required_angular_distance = utils.calculate_necessary_yaw(self._base._current_odom.pose.pose.orientation, goal.orientation)
                
                #n_t_t = math.pi / 2 - utils.angular_diff(self._base._current_odom.pose.pose.position, goal.position)

                required_angular_distance = required_angular_distance - n_t_t

                if required_angular_distance < - math.pi:
                    required_angular_distance += 2 * math.pi

                print 'REQ_ANG_DIST: {}'.format(required_angular_distance)

                rotate_speed = 1.0
                if required_angular_distance > 0:
                    rotate_speed *= -1

                if abs(required_angular_distance) > 0.6:
                    self._base.move(0, rotate_speed)
                elif abs(required_angular_distance) > 0.35:
                    rotate_speed *= 0.6
                    self._base.move(0, rotate_speed)
                elif abs(required_angular_distance) > 0.04:
                    rotate_speed *= 0.3
                    self._base.move(0, rotate_speed)
                else:
                    print 'Done Turning'
                    speed = 0.5       
                    self._base.stop
                    time.sleep(0.5)    
                    state = 'move'

            if state == 'move':
                # Compute how far we have moved and compare that to desired_distance
                # TODO:  Make sure that the robot has the ability to drive backwards if it overshoots
                check_angle += 1
                curr_distance = utils.calc_minkowski_distance(start.pose.pose.position, self._base._current_odom.pose.pose.position, 2)
                delta = curr_distance - desired_distance
                if curr_distance < desired_distance:
                    print 'delta: {}'.format(delta)
                    if abs(delta) < 0.5:
                        speed = 0.2
                    self._base.move(speed, 0)
                    print 'check_turn: {}'.format(check_angle)
                    if check_angle % 80 == 0:
                        state = 'turn'
                else:
                    self._base.stop()
                    time.sleep(0.5)
                    state = None
            rospy.sleep(0.1)
