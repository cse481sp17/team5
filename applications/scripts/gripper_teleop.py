#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from fetch_api import Arm, Gripper
import gripper_grasp_utils
import copy

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self.armPose = PoseStamped()
        self.armPose.header.frame_id = 'base_link'

    def start(self):
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = 'base_link'
        poseStamped.pose = Pose(orientation=Quaternion(0,0,0,1))
        gripper_im = gripper_grasp_utils.getMarkersFromPose(poseStamped, false)
        self._im_server.insert(gripper_im, self.handle_feedback)
        self._im_server.applyChanges()
        
    def handle_feedback(self, feedback):
        if feedback.event_type is InteractiveMarkerFeedback.MENU_SELECT:
            entry_id = feedback.menu_entry_id
            if entry_id is gripper_grasp_utils.GRIPPER_OPEN:
                self._gripper.open()
            elif entry_id is gripper_grasp_utils.GRIPPER_CLOSE:
                self._gripper.close(70)
            elif entry_id is gripper_grasp_utils.GRIPPER_MOVETO:
                self._arm.move_to_pose(self.armPose)
            else:
                rospy.logerr('INVALID COMMAND')
        elif feedback.event_type is InteractiveMarkerFeedback.POSE_UPDATE:
            interactive_marker = self._im_server.get(feedback.marker_name)
            self.armPose.pose = copy.deepcopy(interactive_marker.pose)
            if self._arm.compute_ik(self.armPose) is True:
                for gripper_marker in interactive_marker.controls[0].markers:
                    gripper_marker.color.r = 0.0
                    gripper_marker.color.g = 1.0
            else:
                for gripper_marker in interactive_marker.controls[0].markers:
                    gripper_marker.color.r = 1.0
                    gripper_marker.color.g = 0.0
            self._im_server.erase(feedback.marker_name)
            self._im_server.insert(interactive_marker)
            self._im_server.applyChanges()





class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = 'base_link'
        poseStamped.pose = Pose(orientation=Quaternion(0,0,0,1))
        obj_im = gripper_grasp_utils.getMarkersFromPose(poseStamped, true)
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)

    def handle_feedback(self, feedback):
        if feedback.event_type is InteractiveMarkerFeedback.MENU_SELECT:
            entry_id = feedback.menu_entry_id
            if entry_id is gripper_grasp_utils.GRIPPER_AUTOPICK:
                interactive_marker = self._im_server.get(feedback.marker_name)
                
            elif entry_id is gripper_grasp_utils.GRIPPER_OPEN:
                self._gripper.open()
            elif entry_id is gripper_grasp_utils.GRIPPER_CLOSE:
                self._gripper.close(70)
            elif entry_id is gripper_grasp_utils.GRIPPER_MOVETO:
                self._arm.move_to_pose(self.armPose)
            else:
                rospy.logerr('INVALID COMMAND')
        elif feedback.event_type is InteractiveMarkerFeedback.POSE_UPDATE:
            interactive_marker = self._im_server.get(feedback.marker_name)
            self.armPose.pose = copy.deepcopy(interactive_marker.pose)
            if self._arm.compute_ik(self.armPose) is True:
                for gripper_marker in interactive_marker.controls[0].markers:
                    gripper_marker.color.r = 0.0
                    gripper_marker.color.g = 1.0
            else:
                for gripper_marker in interactive_marker.controls[0].markers:
                    gripper_marker.color.r = 1.0
                    gripper_marker.color.g = 0.0
            self._im_server.erase(feedback.marker_name)
            self._im_server.insert(interactive_marker)
            self._im_server.applyChanges()


def main():
    rospy.init_node('gripper_teleop')
    arm = Arm()
    gripper = Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server',  q_size=2)
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    # auto_pick.start()
    rospy.spin()

if __name__ == '__main__':
    main()