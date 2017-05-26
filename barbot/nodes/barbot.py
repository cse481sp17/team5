#!/usr/bin/env python

import fetch_api
import rospy
import copy
import actionlib

import NavigationServer
import ArmServer

from barbot.srv import *
from barbot.msg import DrinkStatus, DrinkOrder
from robot_controllers_msgs.msg import QueryControllerStatesGoal, QueryControllerStatesAction, ControllerState

#PICKLE_FILE='pose_list_n.p'

def wait_for_time():
    """Wait for simulated time to begin.
        TODO: LOL, did we really never put this in a utils file?
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class Barbot(object):
    """ Barbot represents the Micheal Bot backend application.

    NOTE:
        Barbot instantiation requires time to have begun and does NOT spin up rospy.

    Attributes:
        _drink_orders (Queue): The currently enqueued drink orders
        _is_working (bool): True iff the robot is currently fulfilling drink orders.
        _gripper (Gripper): The fetch gripper.
        _arm (Arm): The fetch arm.
        _nav_server (NavigationServer): The application navigation server.
        _arm_server (ArmServer): The application arm server.
        _controller_client (SimpleActionClient): The controller states action client.
        _drink_order_subscriber (Subscriber): The subscriber listening for front end drink order requests.
        _drink_status_publisher (Publisher): The publisher that publishes drink order statuses to the fron end.

    """

    def __init__(self):
        self._drink_orders = Queue()
        self._is_working = False

        # Initialize Navigation Server.
        self._nav_server = NavigationServer()
        self._nav_server.loadMarkers()

        # Initialize Arm and Gripper
        self._gripper = fetch_api.Gripper()
        self._arm = fetch_api.Arm()
        rospy.logdebug('Gripper and Arm instantiated.');
        self._arm_server = ArmServer()

        # Startup Fetch ARM
        self._controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
        rospy.sleep(0.5)
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        self._controller_client.send_goal(goal)

        # Wait for Fetch ARM
        rospy.loginfo('Waiting for arm to start.')
        self._controller_client.wait_for_result()
        rospy.loginfo('Arm has started..')

        # Instantiate subscription model with front-end
        self._drink_order_subscriber = rospy.Subscriber('/drink_order', DrinkOrder, self.drink_order_handler)   
        self._drink_status_publisher = rospy.Publisher('/drink_status', DrinkStatus, queue_size=1)
    
    @property
    def get_drink_orders(self):
        """:obj:`list` of :obj:`DrinkOrder`: A deep copy of the current drink orders. """
        return list(self._drink_orders.queue)

    @property
    def get_is_working(self):
        """bool: Whether the application is currently working on a drink order."""
        return self._is_working

    # Implicit: @async
    def fulfill_orders(self):
        """ Fulfill all currently open drink orders.

        NOTE:
            This process will run until ALL drink orders on
            in the order queue have been fulfilled or is_working has
            been set to false.
        TODO:
            This is a super shitty way to constrain concurrency.
        """
        if self._drink_orders.empty() or not self._is_working:
            self._is_working = False
            return
        drink_order = self._drink_orders.get_nowait()
        #TODO: Make the drink!
        #NOTE: You NEED to do this asynchronously.

        # navigate to the bar table
        #nav_server.goToMarker(BAR_TABLE)
        # call service to run cpp file
        #rospy.wait_for_service('barbot/move_to_perception')
        #perception_service = rospy.ServiceProxy('barbot/move_to_perception', MoveToPerception)
        # try:
        #     response = perception_service('cup')
        #     arm_server.findGlass(response)

        # except rospy.ServiceException, e:
        #     print 'Service call failed getting cup'
        #     nav_server.goToMarker(HOME)
        #     continue

        # nav_server.goToMarker(HOME)

        # try:
        #     response = perception_service('table')
        #     arm_server.findGlass(response)
        # except rospy.ServiceException, e:
        #     print 'Service call failed getting table'
        #     continue

        self._publish_drink_status(drink_order.id)
        self.fulfill_orders()

    def drink_order_handler(self, message):
        """ Handle a drink order request. 
        
        Args:
            message (DrinkOrder): The drink order message.
        """
        if(message.command == 'make_order') :
            # Enque the drink job. If the robot is not
            # currently working then re-instantiate the pour routine.
            self._drink_queue.put_nowait(message)
            self._publish_drink_status()
            if not self._is_working:
                self._is_working = True
                self.fulfill_orders()
        else:
            rospy.logwarn('Unknown message command %s' % (command))

    def _publish_drink_status(self, completed=None):
        """ Publish the current drink orders status
    
        Args:
            completed: The id of a completed drink (Optional)
        """
        order_ids = []
        for order in self.get_drink_orders():
            order_ids.append(order.id)
        message = DrinkStatus()
        message.orders = order_ids
        message.completed = completed
        self._drink_status_publisher.publish(message)

def main():
    rospy.init_node('barbot', anonymous=True)
    wait_for_time()
    barbot = Barbot();
    rospy.spin()

if __name__ == '__main__':
    main()