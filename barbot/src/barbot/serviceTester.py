#!/usr/bin/env python

from barbot.srv import *
import rospy

def move_to_perception_client():
    print 'waiting'
    rospy.wait_for_service('move_to_perception')
    try:
        move_to_perception = rospy.ServiceProxy('move_to_perception', MoveToPerception)
        resp = move_to_perception('cup')
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('move_to_tester')
    resp1 = move_to_perception_client()
    rospy.loginfo('x = %d, y = %d, z = %d, item = %s', resp1.x, resp1.y, resp1.z, resp1.item)
    rospy.spin()