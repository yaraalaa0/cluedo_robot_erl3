#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from aruco_ros.srv import Aruco, ArucoRequest
from exp_assignment3.srv import Hint, HintResponse

VERBOSE = False


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('hint_server', anonymous=True)
     # topic where we publish
        self.image_pub = rospy.Publisher("output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",
                                       Twist, queue_size=1)

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/robot/camera1/image_raw",
                                           Image, self.callback,  queue_size=1)
                                           
        self.bridge = CvBridge()
        self.hint_server = rospy.Service('/get_hint', Hint, self.get_hint)
        self.aruco_client = rospy.ServiceProxy('/get_aruco_code', Aruco)
        self.callbackFlag = False
        self.hintFlag = False
        self.markerID1 = -1
        self.markerID2 = -1
        self.count = 0
    
    def get_hint(self, data):
        self.hintFlag = False
        self.callbackFlag = True
        self.markerID1 = -1
        self.markerID2 = -1
        while self.hintFlag == False:
            time.sleep(10)
        
        res = HintResponse()
        res.hintID1 = self.markerID1
        res.hintID2 = self.markerID2
        return res
        
        
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        
        if self.callbackFlag == True:
            if VERBOSE:
                print ('received image of type: "%s"' % ros_data.format)

            #### direct conversion to CV2 ####
            #np_arr = np.fromstring(ros_data.data, np.uint8)
            #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
            
            image_np = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
            
            blackLower = (0, 0, 0)
            blackUpper = (255, 255, 20)
            
            blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, blackLower, blackUpper)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            #cv2.imshow('mask', mask)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            # only proceed if at least one contour was found
            if len(cnts) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                #print("Center y = ", center[1])
                # only proceed if the radius meets a minimum size
                if radius > 10 and center[1] > 200:
                    if radius > 110 and center[0] > 300 and center[0] < 500:
                        print("Sending image to Aruco")
                        #self.callbackFlag = False
                        aruco_req = ArucoRequest()
                        aruco_req.image = ros_data
                        result = self.aruco_client(aruco_req)
                        if result.markerID != -1 and result.markerID <= 40:
                            if self.markerID1 == -1:
                                self.markerID1 = result.markerID
                                print("The received first marker ID is ")
                                print(self.markerID1)
                            elif result.markerID != self.markerID1:
                                self.markerID2 = result.markerID
                                print("The received second marker ID is ")
                                print(self.markerID2)
                                self.hintFlag = True
                                self.callbackFlag = False
                            else:
                                # continue exploring
                                vel = Twist()
                                vel.angular.z = 0.5
                                self.vel_pub.publish(vel)
                                time.sleep(2)
                        
                    else:
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                        vel = Twist()
                        vel.angular.z = 0.002*(400-center[0])
                        vel.linear.x = -0.001*(radius-120)
                        self.vel_pub.publish(vel)
                
                elif radius <= 10 and center[1] > 200:
                    vel = Twist()
                    vel.angular.z = 0.002*(400-center[0])
                    vel.linear.x = 0.3
                    self.vel_pub.publish(vel)
                else:
                    vel = Twist()
                    vel.angular.z = 0.5
                    self.vel_pub.publish(vel)

            else:
                vel = Twist()
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)

            # update the points queue
            # pts.appendleft(center)
            cv2.imshow('window', image_np)
            cv2.waitKey(2)
            
            

            # self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
