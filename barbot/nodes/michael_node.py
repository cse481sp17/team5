#!/usr/bin/env python

import fetch_api
import rospy
import pickle
import copy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import PoseNames
from map_annotator.msg import UserAction
import geometry_msgs.msg
from barbot.srv import *
from pose_executable import *

PICKLE_FILE='pose_list_n.p'
PICKLE_FILE_PUT_DISPENSER='put_dispenser.p'
PICKLE_FILE_PUT_CLIENT='put_client.p'
PICKLE_FILE_DISPENSE='dispenser.p'
BAR_TABLE='bar_table'
HOME='home'
OFFSET_X = 0.15
OFFSET_Z = 0.10
GRIPPER_OFFSET = 0.171
DISPENSE_TIME = 1.0
#current_amcl = None

class ArmServer(object):
    def __init__(self):
        self._grip = fetch_api.Gripper()
        self._arm = fetch_api.Arm()

    def findGlass(self, request):
        current_pose.position.x = request.x - GRIPPER_OFFSET
        current_pose.position.y = request.y
        current_pose.position.z = request.z



class NavigationServer(object):
    def __init__(self):
        self._target_pose_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
        self.pose_names_list = {}

    
    def loadMarkers(self):
        try:
            self.pose_names_list = pickle.load(open(PICKLE_FILE, "rb"))
        except:
            print 'Pickle File Empty'
            self.pose_names_list = {}
            pickle.dump(self.pose_names_list, open(PICKLE_FILE, "wb" ) )
    
    def saveMarker(self, name, position):
        self.pose_names_list[name] = position
        pickle.dump(self.pose_names_list, open(PICKLE_FILE, "wb" ))
    
    def goToMarker(self, name):
        target_pose_marker = self.pose_names_list[name]
        message = geometry_msgs.msg.PoseStamped()
        message.header.frame_id = "map"
        message.pose = target_pose_marker
        self._target_pose_pub.publish(message)

    def deleteMarker(self, name):
        del self.pose_names_list[name]
        pickle.dump(self.pose_names_list, open(PICKLE_FILE, "wb" ))
        
nav_server = None
target_pose_pub = None
pose_names_pub = None
pose_marker_server = None
current_pose = None


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def grabGlass():
    gripper.open()
    goal = PoseStamped()
    goal.header.frame_id = 'base_link'
    goal.pose = copy.deepcopy(current_pose)
    goal.pose.position.x -= OFFSET_X
    arm.move_to_pose(goal)
    goal.pose.position.x += OFFSET_X
    arm.move_to_pose(goal)
    gripper.close()
    goal.pose.position.z += OFFSET_Z
    arm.move_to_pose(goal)

def handle_user_actions(message):
        name = message.name
        if message.command == 'make_drink':
            if name in nav_server.pose_names_list:
            # navigate to the bar table
                nav_server.goToMarker(BAR_TABLE)
                # find a nearest glass on the table and catch it
                grabGlass()
                # find the dispenser and set the glass correctly on it
                load_faducial_actions(PICKLE_FILE_PUT_DISPENSER)
                # dispense drink into the glass for a while
                load_faducial_actions(PICKLE_FILE_DISPENSE)
                # catch the glass again
                grabGlass()
                # navigate back to the client table
                nav_server.geToMarker(name)
                # set the glass on the client N table
                load_faducial_actions(PICKLE_FILE_PUT_CLIENT)
        else:
            pass


def transform_to_pose(matrix):
    pose = Pose()
    quat_from_mat = tft.quaternion_from_matrix(matrix)
    pose.orientation = Quaternion(quat_from_mat[0], quat_from_mat[1], quat_from_mat[2], quat_from_mat[3])
    x = matrix[0][3]
    y = matrix[1][3]
    z = matrix[2][3]
    pose.position = Point(x, y, z)
    return pose

def makeMatrix(pose):
    orien = pose.orientation
    base_link_mat = tft.quaternion_matrix([orien.x, orien.y, orien.z, orien.w])
    base_link_mat[0][3] = pose.position.x
    base_link_mat[1][3] = pose.position.y
    base_link_mat[2][3] = pose.position.z
    base_link_mat[3][0] = 0
    base_link_mat[3][1] = 0
    base_link_mat[3][2] = 0
    base_link_mat[3][3] = 1
    return base_link_mat

def load_faducial_actions(fileName):
    pose_actions = None
    try:
        pose_actions = pickle.load(open(fileName, "rb"))
        print '{} loaded.'.format(fileName)
    except:
        print '{} could not be loaded.'.format(fileName)
        usage()
        return

    count = 0

    # Run through each of the actions
    for pose_action in pose_actions:
        count += 1
        print 'Performing action.'
        if pose_action.actionType == PoseExecutable.OPEN:
            print 'Opening the gripper'
            gripper.open()
        elif pose_action.actionType == PoseExecutable.CLOSE:
            print 'Closing the gripper'
            gripper.close()
        elif pose_action.actionType == PoseExecutable.MOVETO:
            print 'Moving to location.'
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            if pose_action.relativeFrame == 'base_link':
                pose_stamped.pose = pose_action.pose
            else:
                for marker in reader.markers:
                    if pose_action.relativeFrame == marker.id:
                        wrist2 = makeMatrix(pose_action.pose) 
                        tag = makeMatrix(pose_action.arPose)  
                        tag2 = tf.transformations.inverse_matrix(tag)
                        result = np.dot(tag2, wrist2)
                        result2 = np.dot(makeMatrix(marker.pose.pose), result)

                        pose_stamped = PoseStamped()
                        pose_stamped.header.frame_id = "base_link"
                        pose_stamped.pose = transform_to_pose(result2)
            error = arm.move_to_pose(pose_stamped, allowed_planning_time=40, num_planning_attempts=20)
            if error is not None:
                print 'Error moving to {}.'.format(pose_action.pose)
                
            if fileName is PICKLE_FILE_DISPENSE and count == 2:
                rospy.sleep(DISPENSE_TIME)
        else:
            print 'invalid command {}'.format(pose_action.action)

def main():
    global pose_marker_server
    global nav_server
    global target_pose_pub
    global pose_names_pub
    global current_pose
    current_pose = geometry_msgs.msg.Pose(orientation=geometry_msgs.msg.Quaternion(0,0,0,1))

    rospy.init_node('action_node')
    wait_for_time()

    pose_marker_server = InteractiveMarkerServer('simple_marker')
    pose_names_pub = rospy.Publisher('/pose_names', PoseNames, queue_size=10, latch=True)

    nav_server = NavigationServer()
    # Create
    nav_server.loadMarkers()
    
    arm_server = ArmServer()

    message = PoseNames()
    message.names = nav_server.pose_names_list
    pose_names_pub.publish(message) 

    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)
    controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    rospy.sleep(0.5)
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.RUNNING
    goal.updates.append(state)
    controller_client.send_goal(goal)

    print 'Waiting for arm to start.'
    controller_client.wait_for_result()
    print 'Arm has been started.'


    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()

    print 'Arm and gripper instantiated.'

    # handle user actions
    user_actions_sub = rospy.Subscriber('/user_actions', UserAction, handle_user_actions)   

    arm_service = rospy.Service('barbot/arm_to_pose', ArmToPose, arm_server.findGlass)

    rospy.spin()

if __name__ == '__main__':
    main()