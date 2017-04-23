#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from fetch_api import DestinationMarker, Driver, Base

if __name__ == "__main__":
    rospy.init_node('simple_marker')
    server = InteractiveMarkerServer('simple_marker')
    driver = Driver(Base())
    marker1 = DestinationMarker(server, 2, 2, 'dest1', driver)
    marker2 = DestinationMarker(server, 1, 0, 'dest2', driver)
    marker3 = DestinationMarker(server, 3, -1, 'dest3', driver)
    driver.start()
    rospy.spin()