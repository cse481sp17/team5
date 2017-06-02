#! /usr/bin/env python                                                                                 
                                                                                                       
import fetch_api                                                                                       
from joint_state_reader import JointStateReader
import rospy                  
                              
                       
def wait_for_time():   
    """Wait for simulated time to begin.
    """                
    while rospy.Time().now().to_sec() == 0:                                                            
        pass                                                                                           
                                                                                                       
                                                                                                       
def main():                                                                                            
    rospy.init_node('joint_reader_demo')                                                               
    wait_for_time()                                                                                    
    argv = rospy.myargv()                                                                              
    reader = JointStateReader()
    rospy.sleep(0.5)
    names = fetch_api.ArmJoints.names()
    arm_vals = reader.get_joints(names)
    for k, v in zip(names, arm_vals):
        print '{}\t{}'.format(k, v)

    armList = ['shoulder_pan_joint','shoulder_lift_joint','upperarm_roll_joint','elbow_flex_joint','forearm_roll_joint','wrist_flex_joint','wrist_roll_joint']

    value = reader.get_joints(armList)
    # print value
    # value = reader.get_joint('l_gripper_finger_joint')
    print value


                      
                      
if __name__ == '__main__':
    main()