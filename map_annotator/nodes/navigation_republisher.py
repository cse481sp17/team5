#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from map_annotator.msg import PoseNames
import pickle

PICKLE_FILE='pose_list_n.p'
map_list_data = {}
current_amcl = None


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    global map_list_data
    rospy.init_node('navigation_republisher')
    wait_for_time()

    pose_names_pub = rospy.Publisher('/pose_names', PoseNames, queue_size=10, latch=True)
    rospy.sleep(1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            map_list_data = pickle.load(open(PICKLE_FILE, "rb"))
        except:
            print 'Pickle File Empty'
            map_list_data = {}
            pickle.dump(map_list_data, open(PICKLE_FILE, "wb" ) )
        message = PoseNames()
        message.names = map_list_data
        pose_names_pub.publish(message)

        rate.sleep()


if __name__ == '__main__':
    main()