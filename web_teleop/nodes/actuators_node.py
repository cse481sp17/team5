#!/usr/bin/env python

import fetch_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse, SetHeadPanTilt, SetHeadPanTiltResponse, SetGripper, SetGripperResponse, SetArm, SetArmResponse

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._head = fetch_api.Head()
        self._grip = fetch_api.Gripper()
        self._arm = fetch_api.Arm()

    def handle_set_torso(self, request):
        self._torso.set_height(request.height)
        return SetTorsoResponse()
    
    def handle_set_head_pan(self, request):
        self._head.pan_tilt(request.pan, request.tilt)
        return SetHeadPanTiltResponse()

    def handle_set_gripper(self, request):
        if request.position > 0.0:
            self._grip.open()
        else:
            self._grip.close()
        return SetGripperResponse()
    
    def handle_set_arm(self, request):
        result = [request.armShoulderPan, request.armShoulderLift, request.armUpperarmRoll, request.armElbowFlex, request.armForearmRoll, request.armWristFlex, request.armWristRoll]
        #vals = [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0]
        self._arm.move_to_joints(fetch_api.ArmJoints.from_list(result))
        return SetArmResponse()


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    # Torso
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,server.handle_set_torso)
    # HEad
    head_pan_tilt_service = rospy.Service('web_teleop/set_head_pan_tilt', SetHeadPanTilt, server.handle_set_head_pan)
    #Gripper
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper,server.handle_set_gripper)
    #Arm
    arm_service = rospy.Service('web_teleop/set_arm', SetArm, server.handle_set_arm)
    rospy.spin()


if __name__ == '__main__':
    main()