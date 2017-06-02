#! /usr/bin/env python

import fetch_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass



def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    argv = rospy.myargv()
    DISCO_POSES = [[-1.605528329547318, 1.41720603380179, 2.018610841968549, 1.5522558117738399, -1.5635699410855368, 0.7653977094751401, -1.3914909133500242]]

    #torso = fetch_api.Torso()
    #torso.set_height(fetch_api.Torso.MAX_HEIGHT)

    arm = fetch_api.Arm()
    for vals in DISCO_POSES:
        arm.move_to_joints(fetch_api.ArmJoints.from_list(vals))


if __name__ == '__main__':
    main()
