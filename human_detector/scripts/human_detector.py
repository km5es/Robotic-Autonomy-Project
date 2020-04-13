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

# Define a callback for the Image message
def image_callback(img_msg):
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
    #contours,hierarchy = cv2.findContours(mask_noNoise, 1, 2)
    _, contours, _= cv2.findContours(mask_noNoise, 1, 2)

    # Calc coordiantes of human in image by calculating the centroid of the contour
    M = cv2.moments(contours[0])
    x = int(M['m10']/M['m00'])
    y = int(M['m01']/M['m00'])

    rospy.loginfo("Hello World, X= "+str(x)+"; Y="+str(y))
    #!!!!!!!!!
    #Get depth information here
    #!!!!!!!!!

    #!!!!!!!!!
    #Convert coordiantes to bearing here
    #!!!!!!!!!

    #!!!!!!!!!
    #Call EKF service here
    # Position of robot
    Sx0 = 0
    Sx1 = 0
    # Motion of robot
    Su0 = 0
    Su1 = 0
    # Position of target
    Tx0 = 4
    Tx1 = 4
    # Motion and angle of target (differential drive human model)
    vri = 1.5
    vli = 1.5
    thk = 0 #thkr
    # Tm and TS (initialize, then use previous values)
    Tm0 = 0 #Tmr0
    Tm1 = 0 #Tmr1
    TS0 = 5 #TSr0
    TS1 = 0 #TSr1
    TS2 = 0 #TSr2
    TS3 = 5 #TSr3

    rospy.wait_for_service('human_prob_motion')
    try:
	human_motion = rospy.ServiceProxy('human_prob_motion', HumanProbMotion)
        resp = human_motion(Sx0, Sx1, Su0, Su1, Tx0, Tx1, vri, vli, thk, Tm0, Tm1, TS0, TS1, TS2, TS3)
        
        print "%s, %s, %s, %s, %s, %s, %s, %s, %s"%(resp.Txr0, resp.Txr1, resp.thk, resp.Tmr0, resp.Tmr1, resp.TSr0, resp.TSr1, resp.TSr2, resp.TSr3)        

        # Position of robot
        Sxr0 = resp.Sxr0
        Sxr1 = resp.Sxr1
        # Position and orientation of target
        Txr0 = resp.Txr0
        Txr1 = resp.Txr1
        thkr = resp.thk
        # Prob model Tm and TS
        Tmr0 = resp.Tmr0
        Tmr1 = resp.Tmr1
        TSr0 = resp.TSr0
        TSr1 = resp.TSr1
        TSr2 = resp.TSr2
        TSr3 = resp.TSr3
   
    except rospy.ServiceException, e:
    	print "Service call failed: %s"%e
    #!!!!!!!!!

    #!!!!!!!!!
    #Publish result
    #!!!!!!!!!



# Initialize the ROS Node, allow multiple nodes to be run with this name
rospy.init_node('human_detector', anonymous=True)

# Initialize the CvBridge class
bridge = CvBridge()

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

# Set up service
#human_prob_motion = rospy.ServiceProxy('human_prob_motion', HumanProbMotion)

#!!!!!!!!!
#Set up publisher
#!!!!!!!!!

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
