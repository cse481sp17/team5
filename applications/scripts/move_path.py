#!/usr/bin/env python

import fetch_api
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
import time

import sys, select, termios, tty

class NavPath(object):
    def __init__(self):
        self.movePoints = []
            
    def callback(self, msg):
        rospy.loginfo(msg)
        if len(self.movePoints) == 0:
            self.movePoints.append(msg.pose.pose.position)

        if msg.pose.pose.position.x - self.movePoints[-1].x > 0.1:
            self.movePoints.append(msg.pose.pose.position)

def create_line_marker(text):
    marker = Marker()
    marker.type = Marker.LINE_STRIP
    marker.lifetime(100)
    marker_publisher.publish(marker)


def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass


def main():
    # ...setup stuff...
    rospy.init_node('move_path')
    #wait_for_time()
    wait_for_time()

    rospy.sleep(0.5)                                        

    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()

if __name__ == '__main__':
  main()