#!/usr/bin/env python

from barbot.srv import *
import rospy

def move_to_perception_client(item):
    print 'waiting'
    rospy.wait_for_service('move_to_perception')
    try:
        move_to_perception = rospy.ServiceProxy('move_to_perception', MoveToPerception)
        resp = move_to_perception(item)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('move_to_tester')
    resp1 = move_to_perception_client(MoveToPerceptionRequest.CUP)
    rospy.loginfo('x = %f, y = %f, z = %f, item = %s', resp1.x, resp1.y, resp1.z, resp1.item)

    # resp1 = move_to_perception_client(MoveToPerceptionRequest.TABLE)
    # rospy.loginfo('x = %f, y = %f, z = %f, item = %s', resp1.x, resp1.y, resp1.z, resp1.item)
    rospy.spin()