#!/usr/bin/env python

import sys
import rospy
import std_srvs
from std_srvs.srv import Empty

def global_localization_client():
    rospy.wait_for_service('global_localization')
    try:
        global_localization = rospy.ServiceProxy('global_localization', Empty) #get a function handle from ROS
        global_localization() #this is the service to call
        return
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s"%sys.argv[0]

if __name__ == "__main__": #main function
    if len(sys.argv) > 3:
        print usage()
        sys.exit(1)
    print "Calling global_localization..."
    global_localization_client()