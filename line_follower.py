#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define the HSV range for red color (two ranges to capture red)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red color using the two ranges
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        
        # Combine the two masks to detect red in both ranges
        mask = cv2.bitwise_or(mask1, mask2)
        
        h, w, d = image.shape
        search_top = int(3 * h / 4)
        search_bot = int(search_top + 20)
        
        # Mask out the upper and lower parts of the image to focus on the middle
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)  # Red circle at the center
            
            # Calculate error (how far the center of the red object is from the middle)
            err = cx - w / 2
            self.twist.linear.x = 0.75  # Move forward
            self.twist.angular.z = -float(err) / 1000  # Rotate based on error
            
            self.cmd_vel_pub.publish(self.twist)
        
        # Show the image with the detected red object
        cv2.imshow("window", image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('follower')
    follower = Follower()
    rospy.spin()

