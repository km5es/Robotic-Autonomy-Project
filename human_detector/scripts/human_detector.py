#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
import numpy as np
from sensor_msgs.msg import Image
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math

# Human prob motion service
from human_prob_motion.srv import *
import math
from std_msgs.msg._Float32 import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def get_depth(depth_msg):
    # Try to convert depth image to CV readable image.
    # Initialize the CvBridge class
    global depth
    global x
    global y
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(depth_msg, "32FC1")
    depth = cv_image[y, x]    # distance to tracked pixels

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
    lowerBoundary = np.array([10, 50, 50])
    upperBoundary = np.array([40, 255, 255])

    # Mask
    mask = cv2.inRange(img_hsv, lowerBoundary, upperBoundary)

    # Removing noise by opening (erosion followed by dilation)
    kernel = np.ones((5,5),np.uint8)
    mask_noNoise = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours
    _, contours, _ = cv2.findContours(mask_noNoise, 1, 2)

    # Calc coordiantes of human in image by calculating the centroid of the contour
    M = cv2.moments(contours[0])
    x = int(M['m10']/M['m00'])
    y = int(M['m01']/M['m00'])

    #rospy.loginfo("X= "+str(x)+"; Y="+str(y))

    #!!!!!!!!!
    #Convert coordiantes to bearing here
    # Kinect has a field of view of 62x48.6 and resolution of 640x480
    bearing_deg = (31.0/320.0)*(x-320.0)
    bearing_rad = (math.pi/180.0)*bearing_deg
    #!!!!!!!!!

    #rospy.loginfo("bearing = "+str(bearing_rad))

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
    Tx0 = Sx0 + depth*math.sin(robot_bearing+bearing_rad)
    #Tx0 = Sx0+depth*math.cos(bearing_rad)*math.sin(robot_bearing)+depth*math.sin(bearing_rad)*math.cos(robot_bearing)
    Tx1 = Sx1 + depth*math.cos(robot_bearing+bearing_rad)
    #Tx1 = Sx1+depth*math.cos(bearing_rad)*math.cos(robot_bearing)+depth*math.sin(bearing_rad)*math.sin(robot_bearing)
    # Motion and angle of target (differential drive human model)
    vri = 0 #math.sqrt((pow(Tx0-Txs0, 2))+(pow(Tx1-Txs1, 2)))/0.033
    vli = 0 #math.sqrt((pow(Tx0-Txs0, 2))+(pow(Tx1-Txs1, 2)))/0.033
    thk = math.atan((Txs0-Tx0)/(Txs1-Tx1)) #thkr
    # Tm and TS (initialize, then use previous values)
    Tm0 = Tms0 #Tmr0
    Tm1 = Tms1 #Tmr1
    TS0 = TSs0 #TSr0
    TS1 = TSs1 #TSr1
    TS2 = TSs2 #TSr2
    TS3 = TSs3 #TSr3

    rospy.wait_for_service('human_prob_motion')
    if math.isnan(depth) == False:
    
      try:
          human_motion = rospy.ServiceProxy('human_prob_motion', HumanProbMotion)
          resp = human_motion(Sx0, Sx1, Su0, Su1, Tx0, Tx1, vri, vli, thk, Tm0, Tm1, TS0, TS1, TS2, TS3)

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

	  # Differential Entropy
	  # H(X) = ln(sqrt(((2*pi*e)^nx)|TS|))
          entropy = math.log((math.pow((2*math.pi*math.e),2))*((TSs0*TSs3)-(TSs1*TSs2)))
          #print("Entropy: %s" %entropy)
          track_EKF_f.write("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n"%(depth, resp.Txr0, resp.Txr1, resp.thk, resp.Tmr0, resp.Tmr1, resp.TSr0, resp.TSr1, resp.TSr2, resp.TSr3, entropy))

      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

    #!!!!!!!!!
    #Publish result
    #!!!!!!!!!
    ## publish to /follower/base_controller/cmd_vel for control

def get_human_odom(msg):
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    or_x = msg.pose.pose.orientation.x 
    or_y = msg.pose.pose.orientation.y
    or_z = msg.pose.pose.orientation.z
    track_human_f.write(str(pos_x)+"\t"+str(pos_y)+"\t"+str(or_x)+"\t"+str(or_y)+"\t"+str(or_z)+"\n")
#    print msg.pose.pose

def get_robot_odom(msg):
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    or_x = msg.pose.pose.orientation.x 
    or_y = msg.pose.pose.orientation.y 
    or_z = msg.pose.pose.orientation.z
    track_robot_f.write(str(pos_x)+"\t"+str(pos_y)+"\t"+str(or_x)+"\t"+str(or_y)+"\t"+str(or_z)+"\n")
#    print msg.pose.pose

def main():
    global track_EKF_f
    global track_human_f
    global track_robot_f
    # Initialize the ROS Node, allow multiple nodes to be run with this name
    rospy.init_node('human_detector', anonymous=True)
    ## Subscribers to camera and odom topics
    sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, get_depth)
    robot_odom = rospy.Subscriber("/follower/base_controller/odom", Odometry, get_robot_odom)
    human_odom = rospy.Subscriber("/kbot/base_controller/odom", Odometry, get_human_odom)

    ### Data collection. NOTE: files will be saved in /home/$USER/ directory.
    track_EKF_f = open("track_EKF.csv", "w")
    track_human_f = open("track_human.csv", "w")
    track_robot_f = open("track_robot.csv", "w")
    ## Column headings for sanity
    track_EKF_f.write("depth\tTxr0\tTxr1\tthk\tTmr0\tTmr1\tTSr0\tTSr1\tTSr2\tTSr3\tEntropy\n")
    track_human_f.write("Position_x\tPosition_y\tOrientation_x\tOrientation_y\tOrientation_z\n")
    track_robot_f.write("Position_x\tPosition_y\tOrientation_x\tOrientation_y\tOrientation_z\n")
    
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

    x = 0
    y = 0
    depth = 1
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass

