#!/usr/bin/env python
from threading import Thread
import fetch_api
import time
import signal
import rospy
import pickle
import copy
import actionlib
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
import geometry_msgs.msg
from barbot.srv import *
from barbot.msg import *
from botNavigation import NavigationServer
from botArm import ArmServer
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState

PICKLE_FILE='pose_list.p'
BAR_TABLE='bar_table'
HOME='home'

WORKING=None
orders=[]
nav_server = None
arm_server = None
sound_handler = None

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def handle_user_actions(message):
    # add drink type and id 
    global WORKING
    global orders
    print 'I got a message'
    print message.command
    print message.id
    if message.command == DrinkOrder.MAKE_ORDER:
        print 'appended'
        orders.append([message.id, message.ammount, message.type])
        publish_drink_status()
    if message.command == DrinkOrder.CANCEL_ORDER:
        print ' order cancelled '
        orders.remove([message.id, message.ammount, message.type])
    print orders

def query_orders():
    if len(orders) == 0:
        return
    handle_make_drink()
    

def handle_make_drink():
    global WORKING
    global orders
    if  WORKING is None:
        curr = orders.pop(0)
        WORKING = curr[0]
        amount = int(curr[1]) / 3.0
        orderType = curr[2]
        sound_handler.say('Pouring you a {}'.format(orderType))

        print 'moving to bar table'
        arm_server.lookup()
        # nav_server.goToMarker('middle')
        nav_server.goToMarker(BAR_TABLE)
        # call service to run cpp file
        rospy.sleep(0.5)
        arm_server.lookdown()
        arm_server.set_prepose()
        rospy.sleep(2)
        rospy.wait_for_service('move_to_perception')
        perception_service = rospy.ServiceProxy('move_to_perception', MoveToPerception)
        try:
            response = perception_service('cup')
            print 'I got the response and now try to find a glass'
            rospy.loginfo('x = %f, y = %f, z = %f, item = %s', response.x, response.y, response.z, response.item)

            if (response.x == 0.0 and response.y == 0.0 and response.z == 0.0) :
                rospy.sleep(0.5)
                rospy.wait_for_service('move_to_perception')
                perception_service = rospy.ServiceProxy('move_to_perception', MoveToPerception)
                response = perception_service('cup')
                print 'I got the response and now try to find a glass'
                rospy.loginfo('x = %f, y = %f, z = %f, item = %s', response.x, response.y, response.z, response.item)
            arm_server.findGlass(response, amount)

        except rospy.ServiceException, e:
            print 'Service call failed getting cup'
            arm_server.lookup()
            nav_server.goToMarker(HOME)
            publish_drink_status(WORKING)
            WORKING = None
            return

        print 'moving to home'
        arm_server.lookup()
        nav_server.goToMarker(HOME)
        rospy.sleep(1)
        arm_server.lookdown()
        rospy.sleep(1)
        error = True
        count = 0
        while count < 3:
            try:
                response = perception_service('table')
                error = arm_server.findGlass(response)
                break
            except rospy.ServiceException, e:
                count += 1
        if count == 3 or not error:
            publish_drink_status(WORKING)
            WORKING = None
            return
        # send back id and "done"
        publish_drink_status(WORKING)
        print 'Drink completed.'
        sound_handler.say('Here is your drink. Enjoy!')
        WORKING = None        
    else:
        # send         orders.pop()back "still working"
        publish_drink_status(WORKING)
        print 'still working'

def publish_drink_status(drink_id=None):
    global orders
    send_orders = []
    for order in orders:
        send_orders.append(order[0])
    if drink_id == None:
        drink_id = 'NAN'

    drink_status_pub = rospy.Publisher('/drink_status', DrinkStatus, queue_size=10, latch=True)
    message = DrinkStatus()
    message.orders = send_orders
    message.completed = drink_id
    drink_status_pub.publish(message)

def signal_handler(signal, frame):
    print('Exiting Safely!')
    sys.exit(0)

def main():
    global nav_server
    global WORKING
    global arm_server
    global orders
    global sound_handler
    sound_handler = SoundClient()
    orders = []
    WORKING = None


    rospy.init_node('barbot_controller_node')
    wait_for_time()
    nav_server = NavigationServer()
    nav_server.loadMarkers()

    gripper = fetch_api.Gripper()
    arm = fetch_api.Arm()
    arm_server = ArmServer()
    print 'Arm server instantiated.'

    # MOVE TO MICHAEL_NODE
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

    arm_server.lookup()

    arm_server.set_init_poses()
    arm_server.set_prepose()
    # nav_server.goToMarker(BAR_TABLE)
    print 'going home'

    nav_server.goToMarker(HOME)

    print 'please start to order now'
    sound_handler.say('You may now order.')

    # # handle user actions
    drink_order_sub = rospy.Subscriber('/drink_order', DrinkOrder, handle_user_actions)
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        query_orders()
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()
