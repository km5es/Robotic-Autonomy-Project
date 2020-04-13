#!/usr/bin/env python

# make node executable:
# >>> chmod +x scripts/test_client.py

import sys
import rospy
from human_prob_motion.srv import *

def human_motion_client(x, y):
    rospy.wait_for_service('human_prob_motion')
    try:
        human_motion = rospy.ServiceProxy('human_prob_motion', HumanProbMotion)
        resp = human_motion(0, 0, 0, 0, x, y, 1.5, 1.5, 0, 0, 0, 5, 0, 0, 5)
        print "%s, %s, %s, %s, %s, %s, %s, %s, %s"%(resp.Txr0, resp.Txr1, resp.thk, resp.Tmr0, resp.Tmr1, resp.TSr0, resp.TSr1, resp.TSr2, resp.TSr3)
        return resp.Sxr0
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, human_motion_client(x, y))


