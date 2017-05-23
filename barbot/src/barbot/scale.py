#!/usr/bin/env python
# TEAM 5
# CSE 481C
# Carl Ross, Mahir Kothary, Xukai Liu, Becky Leslie, Ling Jiaowang

import rospy
import geometry_msgs.msg
import sys
import subprocess
import time
from os import listdir

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    # rospy.Publisher('bar_scale', geometry_msgs.msg.PoseStamped, queue_size=10)
    # bashCommand = "cat /dev/hidraw2 | head -c 7"
    print('MAIN');
    while(True):
        # process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        # output, error = process.communicate()
        # print(output)
        print(os.listdir('/'));
        # with open("/dev/hidraw2") as myfile:
        #      head = [next(myfile) for x in xrange(7)]
        # print head
        time.sleep(0.1)

if __name__ == '__main__':
    main()