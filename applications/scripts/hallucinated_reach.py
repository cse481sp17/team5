#! /usr/bin/env python

from geometry_msgs.msg import PoseStamped, Pose, Quaternion
import fetch_api
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
import gripper_grasp_utils
import copy
GRIPPER_OFFSET = 0.171


def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node('hallucinated_demo')
    wait_for_time()
    argv = rospy.myargv()

    torso = fetch_api.Torso()
    torso.set_height(0.4)
    

    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose = Pose(orientation=Quaternion(0,0,0,1))
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    arm = fetch_api.Arm()
    arm.move_to_pose(start)
    success = []
    reader = ArTagReader()

    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback) # Subscribe to AR tag poses, use reader.callback


    while len(reader.markers) == 0:
        rospy.sleep(0.1)

    for marker in reader.markers:
        #TODO: get the pose to move to
        #goal.pose = Pose(orientation=Quaternion(0,0,0,1))
        moveTo = PoseStamped()
        moveTo.header.frame_id = 'base_link'
        moveTo.pose.position = marker.pose.pose.position
        moveTo.pose.position.x -= GRIPPER_OFFSET
        moveTo.pose.orientation = Quaternion(0,0,0,1)

        error = arm.move_to_pose(moveTo, allowed_planning_time=30.0)
        if error is None:
            rospy.loginfo('Moved to marker {}'.format(marker.id))
            success.append(marker.id)
        else:
            rospy.logwarn('Failed to move to marker {}'.format(marker.id))
    if len(success) > 1:
        rospy.loginfo('Moved to markers listed above')
    else:
        rospy.logerr('Failed to move to any markers!')



if __name__ == '__main__':
    main()