#!/usr/bin/env python                                                                                  
                                                                                                       
import rospy    
from sensor_msgs.msg import JointState          
from std_msgs.msg import String                                                  
                                                                                                       
class JointStateReader(object):                                                                        
    """Listens to /joint_states and provides the latest joint angles.                                  
                                                                                                       
    Usage:                                                                                             
        joint_reader = JointStateReader()                                                              
        rospy.sleep(0.1)                                                                               
        joint_reader.get_joint('shoulder_pan_joint')                                                   
        joint_reader.get_joints(['shoulder_pan_joint', 'shoulder_lift_joint'])                         
    """                                                                                               
    def __init__(self):
        self.arm_values = []
        self.arm_names = []
        rospy.Subscriber("joint_states", JointState, self.callback)

    def callback(self, x):
        self.arm_names = x.name
        self.arm_values = x.position
                                                                                                       
    def get_joint(self, name):                                                                         
        """Gets the latest joint value.                                                                
                                                                                                       
        Args:                                                                                          
            name: string, the name of the joint whose value we want to read.                           
                                                                                                       
        Returns: the joint value, or None if we do not have a value yet.                               
        """                                                                                            
        #rospy.logerr('Not implemented.') 
        if name in self.arm_names:
            return self.arm_values[self.arm_names.index(name)]
        return None                                                                   
                                                                                                       
    def get_joints(self, names):                                                                       
        """Gets the latest values for a list of joint names.                    
                                                                                
        Args:                                                                   
            name: list of strings, the names of the joints whose values we want 
                to read.                                                        
                                                                                
        Returns: A list of the joint values. Values may be None if we do not    
            have a value for that joint yet.                                    
        """           
        #rospy.logerr('Not implemented.')        
        result = []   
        for x in names:
            result.append(self.arm_values[self.arm_names.index(x)])                             
        return result

