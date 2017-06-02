#!/usr/bin/env python

import rospy
from barbot.srv import *
from barbot.msg import *


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass
def main():
    rospy.init_node('test_node')
    wait_for_time()

    # handle user actions
    # rospy.wait_for_service('barbot/user_actions')
    # test_action = rospy.ServiceProxy('barbot/user_actions', UserAction)
    # test_action('make_drink', '1')


    drink_status_pub = rospy.Publisher('/drink_order', DrinkOrder, queue_size=10, latch=True)
    rospy.sleep(1)
    message = DrinkOrder()
    message.command = DrinkOrder.MAKE_ORDER
    message.id = '1'
    message.type = 'good'
    message.ammount = '6'
    drink_status_pub.publish(message)

if __name__ == '__main__':
    main()