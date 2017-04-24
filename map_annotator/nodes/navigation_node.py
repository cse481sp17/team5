#!/usr/bin/env python

from fetch_api import PoseMarker
import rospy
from map_annotator.srv import ##
import pickle
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import geometry_msgs.msg
from map_annotator.msg import PoseNames
from map_annotator.msg import UserAction

PICKLE_FILE='pose_list.p'
map_list_data = []
#current_amcl = None

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class NavigationServer(object):
    def __init__(self):
        #self._driver = fetch_api.Driver()
        rospy.init_node('simple_marker')
        pose_marker_server = InteractiveMarkerServer('simple_marker')

    def handle_user_actions(self, message):

        name = message.name
        if message.command == 'create':
            #driver = Driver(Base())
            new_pose_marker = PoseMarker(server, 2, 2, name)
            map_list_data.append(name)

            pickle.dump(map_list_data, open(PICKLE_FILE, "wb" ))

        elif message.command == 'goto':

            if name in map_list_data:
                target_pose_marker = pose_marker_server.get(name)
                message = geometry_msgs.msg.PoseStamped()
                message.header = target_pose_marker.header
                message.pose = target_pose_marker.pose
                target_pose_pub.publish(message)

            else:
                print 'No such pose \'{}\''.format(name):

        elif if message.command == 'delete':
            if name in map_list_data:
                map_list_data.remove(name)
                pickle.dump(map_list_data, open(PICKLE_FILE, "wb" ))
                pose_marker_server.erase(name)        
        else:
            pass


def main():
    global map_list_data

    rospy.init_node('web_teleop_navigation')
    wait_for_time()

    try:
        map_list_data = pickle.load(open(PICKLE_FILE, "rb"))
    except:
        print 'Pickle File Empty'
        map_list_data = {}
        pickle.dump(map_list_data, open(PICKLE_FILE, "wb" ) )

    global target_pose_pub
    target_pose_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)

    server = NavigationServer()
    # Create
    user_actions_sub = rospy.Subscriber('/user_actions', UserAction, server.handle_user_actions)
    #set_name_service = rospy.Service('web_teleop/set_name', SetName, server.handle_set_name)
    
    rospy.spin()


if __name__ == '__main__':
    main()