#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
import numpy as np
from sensor_msgs.msg import Image
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Human prob motion service
from human_prob_motion.srv import *
import math
from std_msgs.msg._Float32 import Float32

def get_depth(depth_msg):
    # Try to convert the ROS Image message to a CV2 Image
    # Initialize the CvBridge class
    global depth
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    #x_pixel = rospy.Subscriber('x_data', Float32)
    depth = cv_image[x, y]    # hard coding pixel coords to test
    rospy.loginfo("The distance to the object is " +str(depth) + " meters.")

# Define a callback for the Image message
def image_callback(img_msg):
    global x, y
    # Initialize the CvBridge class
    bridge = CvBridge()
    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Convert image to HSV color space
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Hue boundaries for orange human
    lowerBoundary = np.array([10,50,50])
    upperBoundary = np.array([40,255,255])

    # Mask
    mask = cv2.inRange(img_hsv, lowerBoundary, upperBoundary)

    # Removing noise by opening (erosion followed by dilation)
    kernel = np.ones((5,5),np.uint8)
    mask_noNoise = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours
    _, contours, _= cv2.findContours(mask_noNoise, 1, 2)

    # Calc coordiantes of human in image by calculating the centroid of the contour
    M = cv2.moments(contours[0])
    x = int(M['m10']/M['m00'])
    y = int(M['m01']/M['m00'])

    rospy.loginfo("Hello World, X= "+str(x)+"; Y="+str(y))

    #!!!!!!!!!
    #Convert coordiantes to bearing here
    # Kinect has a field of view of 62x48.6 and resolution of 640x480
    bearing_deg = (31.0/320.0)*(x-320.0)
    bearing_rad = (math.pi/180.0)*bearing_deg
    #!!!!!!!!!

    #!!!!!!!!!
    #Call EKF service here
    # Initialize and store state of prob model
    # Position of robot
    global Sxs0
    global Sxs1
    # Position and orientation of target
    global Txs0
    global Txs1
    global thks
    # Prob model Tm and TS
    global Tms0
    global Tms1
    global TSs0
    global TSs1
    global TSs2
    global TSs3

    # Position of robot
    Sx0 = 0
    Sx1 = 0
    # Robot bearing (rad)
    robot_bearing = 0
    # Motion of robot
    Su0 = 0
    Su1 = 0
    # Position of target
    Tx0 = Sx0+depth*math.cos(bearing_rad)*math.sin(robot_bearing)+depth*math.sin(bearing_rad)*math.cos(robot_bearing)
    Tx1 = Sx1+depth*math.cos(bearing_rad)*math.cos(robot_bearing)+depth*math.sin(bearing_rad)*math.sin(robot_bearing)
    # Motion and angle of target (differential drive human model)
    vri = 0
    vli = 0
    thk = math.atan((Txs1-Tx1)/(Txs0-Tx0)) #thkr
    # Tm and TS (initialize, then use previous values)
    Tm0 = Tms0 #Tmr0
    Tm1 = Tms1 #Tmr1
    TS0 = TSs0 #TSr0
    TS1 = TSs1 #TSr1
    TS2 = TSs2 #TSr2
    TS3 = TSs3 #TSr3

    rospy.wait_for_service('human_prob_motion')
    try:
        human_motion = rospy.ServiceProxy('human_prob_motion', HumanProbMotion)
        resp = human_motion(Sx0, Sx1, Su0, Su1, Tx0, Tx1, vri, vli, thk, Tm0, Tm1, TS0, TS1, TS2, TS3)
        
        print "%s, %s, %s, %s, %s, %s, %s, %s, %s"%(resp.Txr0, resp.Txr1, resp.thk, resp.Tmr0, resp.Tmr1, resp.TSr0, resp.TSr1, resp.TSr2, resp.TSr3)        

        # Position of robot
        Sxs0 = resp.Sxr0
        Sxs1 = resp.Sxr1
        # Position and orientation of target
        Txs0 = resp.Txr0
        Txs1 = resp.Txr1
        thks = resp.thk
        # Prob model Tm and TS, update state
        Tms0 = resp.Tmr0
        Tms1 = resp.Tmr1
        TSs0 = resp.TSr0
        TSs1 = resp.TSr1
        TSs2 = resp.TSr2
        TSs3 = resp.TSr3

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    #!!!!!!!!!

    #!!!!!!!!!
    #Publish result
    #!!!!!!!!!
    pub_x = rospy.Publisher('x_data', Float32, queue_size=10)
    pub_y = rospy.Publisher('y_data', Float32, queue_size=10)
    pub_x.publish(round(Txs0, 3))
    pub_y.publish(round(Txs1, 3))


def main():
    # Initialize the ROS Node, allow multiple nodes to be run with this name
    rospy.init_node('human_detector', anonymous=True)

    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, get_depth)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    # Initialize prob state
    Sxs0 = 0
    Sxs1 = 0
    Txs0 = 0
    Txs1 = 0
    thks = 0
    Tms0 = 0
    Tms1 = 0
    TSs0 = 5
    TSs1 = 0
    TSs2 = 0
    TSs3 = 5
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass

