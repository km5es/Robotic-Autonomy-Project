#!/usr/bin/env python2.7
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError



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
    contours,hierarchy = cv2.findContours(mask_noNoise, 1, 2)

    # Calc coordiantes of human in image by calculating the centroid of the contour
    M = cv2.moments(contours[0])
    x = int(M['m10']/M['m00'])
    y = int(M['m01']/M['m00'])

    #!!!!!!!!!
    #Get depth information here
    #!!!!!!!!!

    #!!!!!!!!!
    #Convert coordiantes to bearing here
    #!!!!!!!!!

    #!!!!!!!!!
    #Call EKF service here
    #!!!!!!!!!

    #!!!!!!!!!
    #Publish result
    #!!!!!!!!!



# Initialize the ROS Node, allow multiple nodes to be run with this name
rospy.init_node('humanDetector', anonymous=True)

# Initialize the CvBridge class
bridge = CvBridge()

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

# Set up service
human_prob_motion = rospy.ServiceProxy('human_prob_motion', HumanProbMotion)

#!!!!!!!!!
#Set up publisher
#!!!!!!!!!

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()
