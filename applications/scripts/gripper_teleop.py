#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from fetch_api import Arm, Gripper
import gripper_grasp_utils
import copy

GRIPPER_OFFSET = 0.171

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self.armPose = PoseStamped()
        self.validPose = True
        self.armPose.header.frame_id = 'base_link'

    def start(self):
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = 'base_link'
        poseStamped.pose = Pose(orientation=Quaternion(0,0,0,1))
        gripper_im = gripper_grasp_utils.getMarkersFromPose(poseStamped, False)
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
                if self.validPose:
                    self._arm.move_to_pose(self.armPose)
                else:
                    print "Invalid Pose: ", self.armPose
            else:
                rospy.logerr('INVALID COMMAND')
        elif feedback.event_type is InteractiveMarkerFeedback.POSE_UPDATE:
            interactive_marker = self._im_server.get(feedback.marker_name)
            self.armPose.pose = copy.deepcopy(interactive_marker.pose)
            if self._arm.compute_ik(self.armPose) is True:
                for gripper_marker in interactive_marker.controls[0].markers:
                    gripper_marker.color.r = 0.0
                    gripper_marker.color.g = 1.0
                self.validPose = True
            else:
                for gripper_marker in interactive_marker.controls[0].markers:
                    gripper_marker.color.r = 1.0
                    gripper_marker.color.g = 0.0
                self.validPose = False
            #self._im_server.erase(feedback.marker_name)
            self._im_server.insert(interactive_marker)
            self._im_server.applyChanges()

class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self.armPose = PoseStamped()
        self.armPose.header.frame_id = 'base_link'
        self._im_server = im_server

    def start(self):
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = 'base_link'
        poseStamped.pose = Pose(orientation=Quaternion(0,0,0,1))
        obj_im = gripper_grasp_utils.getMarkersFromPose(poseStamped, True)
        self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()


    def handle_feedback(self, feedback):
        if feedback.event_type is InteractiveMarkerFeedback.MENU_SELECT:
            entry_id = feedback.menu_entry_id
            if entry_id is gripper_grasp_utils.GRIPPER_MOVETO:
                interactive_marker = self._im_server.get(feedback.marker_name)
                grasp_gripper = copy.deepcopy(interactive_marker.controls[0].markers[0])
                grasp_gripper.pose.position.x -= GRIPPER_OFFSET

                pre_grasp_gripper = copy.deepcopy(interactive_marker.controls[0].markers[3])
                pre_grasp_gripper.pose.position.x -= GRIPPER_OFFSET
                lift_gripper = copy.deepcopy(interactive_marker.controls[0].markers[6])
                lift_gripper.pose.position.x -= GRIPPER_OFFSET
                self._gripper.open()
                #self.armPose.pose = gripper_grasp_utils.b_in_marker(pre_grasp_gripper, self.armPose.pose)
                #self.armPose.pose.position.x -= GRIPPER_OFFSET
                #self.armPose.pose = pre_grasp_gripper.pose
                goal = PoseStamped()
                goal.header.frame_id = 'base_link'
                goal.pose = gripper_grasp_utils.b_in_marker(pre_grasp_gripper, self.armPose.pose)
                #goal.pose.position.x -= GRIPPER_OFFSET
                self._arm.move_to_pose(goal)
                #print self.armPose.position
                rospy.sleep(0.5)

                #self.armPose.pose = gripper_grasp_utils.b_in_marker(grasp_gripper, self.armPose.pose)
                goal.pose = gripper_grasp_utils.b_in_marker(grasp_gripper, self.armPose.pose)
                #goal.pose.position.x -= GRIPPER_OFFSET
                self._arm.move_to_pose(goal)
                rospy.sleep(0.5)
                #print self.armPose.position

                self._gripper.close(70)

                #self.armPose.pose = gripper_grasp_utils.b_in_marker(lift_gripper, self.armPose.pose)
                #self.armPose.pose.position.x -= GRIPPER_OFFSET
                goal.pose = gripper_grasp_utils.b_in_marker(lift_gripper, self.armPose.pose)
                #goal.pose.position.x -= GRIPPER_OFFSET
                self._arm.move_to_pose(goal)
                #print self.armPose.position
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
            grasp_gripper = interactive_marker.controls[0].markers[0]
            pre_grasp_gripper = interactive_marker.controls[0].markers[3]
            lift_gripper = interactive_marker.controls[0].markers[6]
            self.armPose.pose = copy.deepcopy(interactive_marker.pose)
            computed_pose = PoseStamped()
            computed_pose.header.frame_id = 'base_link'
            computed_pose.pose = gripper_grasp_utils.b_in_marker(grasp_gripper, self.armPose.pose)
            if self._arm.compute_ik(computed_pose) is True:
                for gripper_marker in interactive_marker.controls[0].markers[:3]:
                    gripper_marker.color.r = 0.0
                    gripper_marker.color.g = 1.0
            else:
                for gripper_marker in interactive_marker.controls[0].markers[:3]:
                    gripper_marker.color.r = 1.0
                    gripper_marker.color.g = 0.0
            
            computed_pose.pose = gripper_grasp_utils.b_in_marker(pre_grasp_gripper, self.armPose.pose)
            if self._arm.compute_ik(computed_pose) is True:
                for gripper_marker in interactive_marker.controls[0].markers[3:6]:
                    gripper_marker.color.r = 0.0
                    gripper_marker.color.g = 1.0
            else:
                for gripper_marker in interactive_marker.controls[0].markers[3:6]:
                    gripper_marker.color.r = 1.0
                    gripper_marker.color.g = 0.0
            
            computed_pose.pose = gripper_grasp_utils.b_in_marker(lift_gripper, self.armPose.pose)
            if self._arm.compute_ik(computed_pose) is True:
                for gripper_marker in interactive_marker.controls[0].markers[6:9]:
                    gripper_marker.color.r = 0.0
                    gripper_marker.color.g = 1.0
            else:
                for gripper_marker in interactive_marker.controls[0].markers[6:9]:
                    gripper_marker.color.r = 1.0
                    gripper_marker.color.g = 0.0
            
            self._im_server.erase(feedback.marker_name)
            self._im_server.insert(interactive_marker)
            self._im_server.applyChanges()


def main():
    rospy.init_node('gripper_teleop')
    arm = Arm()
    gripper = Gripper()
    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    rospy.spin()

if __name__ == '__main__':
    main()