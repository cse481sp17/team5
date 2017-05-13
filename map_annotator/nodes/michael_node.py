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

PICKLE_FILE='pose_list_n.p'
PICKLE_FILE_DISPENSER='dispenser.p'
#current_amcl = None

class ArmServer(object):
    def __init__(self):

    def dispense(self):
        try:
            pose_actions = pickle.load(open(PICKLE_FILE_DISPENSER, "rb"))
            print '{} loaded.'.format(PICKLE_FILE_DISPENSER)
        except:
            print '{} could not be loaded.'.format(PICKLE_FILE_DISPENSER)
            usage()
            return



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
        
pose_marker_server = None
nav_server = None
target_pose_pub = None
pose_names_pub = None

class PoseMarker(object):
    def __init__(self, server, name, pose_set):
        # ... Initialization, marker creation, etc. ...
        self.pose = pose_set
        if pose_set == None:
            self.pose = geometry_msgs.msg.Pose()
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "odom"
        int_marker.name = name
        int_marker.description = name
        int_marker.pose = self.pose


        arrow_marker = Marker()
        arrow_marker.type = Marker.ARROW
        arrow_marker.pose.orientation.w = 1
        arrow_marker.scale.x = 0.7
        arrow_marker.scale.y = 0.07
        arrow_marker.scale.z = 0.07
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 0.5
        arrow_marker.color.b = 0.5
        arrow_marker.color.a = 1.0
        arrow_marker.pose.position.z = 0.1

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))

        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.always_visible = True
        control.markers.append(arrow_marker)
        int_marker.controls.append(control)

        #int_marker_control.markers.append(arrow_marker)


        self._server = server
        self._server.insert(int_marker, self._callback)
        self._server.applyChanges()

    def _callback(self, msg):
         # Get the interactive marker given msg.marker_name?
         # See the InteractiveMarkerServer documentation
        interactive_marker = self._server.get(msg.marker_name)
        position = interactive_marker.pose

        if msg.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.pose = position

            nav_server.saveMarker(msg.marker_name, position)
            self._server.applyChanges()

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def handle_user_actions(message):
        name = message.name
        if message.command == 'create':
            new_pose_marker = PoseMarker(pose_marker_server, name, None)
            nav_server.saveMarker(name, new_pose_marker.pose)
            message = PoseNames()
            message.names = nav_server.pose_names_list
            pose_names_pub.publish(message)

        elif message.command == 'goto':

            if name in nav_server.pose_names_list:
                nav_server.goToMarker(name)

            else:
                print 'No such pose \'{}\''.format(name)

        elif message.command == 'delete':
            if name in nav_server.pose_names_list:
                nav_server.deleteMarker(name)
                message = PoseNames()
                message.names = nav_server.pose_names_list
                pose_names_pub.publish(message)
                pose_marker_server.erase(name)
                pose_marker_server.applyChanges()    

        elif message.command == 'make_drink':
            # navigate to the bar table
            nav_server.goToMarker(bar_table_name)
            # find a nearest glass on the table and catch it
            arm_server.findGlass()
            # find the dispenser and set the glass correctly on it
            arm_server.findDispenser()
            # dispense drink into the glass for a while
            arm_server.dispense(dispense_time)
            # catch the glass again
            arm_server.findGlass()
            # navigate back to the client table
            nav_server.geToMarker(client_N)
            # set the glass on the client N table
            arm_server.dropGlass()

        else:
            pass

def main():
    global pose_marker_server
    global nav_server
    global target_pose_pub
    global pose_names_pub
    rospy.init_node('web_teleop_navigation')
    wait_for_time()

    pose_marker_server = InteractiveMarkerServer('simple_marker')
    pose_names_pub = rospy.Publisher('/pose_names', PoseNames, queue_size=10, latch=True)

    nav_server = NavigationServer()
    # Create
    nav_server.loadMarkers()
    for name in nav_server.pose_names_list:
        marker = PoseMarker(pose_marker_server, name, nav_server.pose_names_list[name])
    
    arm_server = ArmServer()

    message = PoseNames()
    message.names = nav_server.pose_names_list
    pose_names_pub.publish(message)
    user_actions_sub = rospy.Subscriber('/user_actions', UserAction, handle_user_actions)    

    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, reader.callback)
    controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    rospy.sleep(1.0)
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



    rospy.spin()


if __name__ == '__main__':
    main()