#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

def move_forward(pub, speed, distance):
    # Create Twist message for forward movement
    move_cmd = Twist()
    move_cmd.linear.x = speed
    distance_traveled = 0.0
    rate = rospy.Rate(10)  # 10 Hz

    # Start time for distance calculation
    start_time = rospy.Time.now().to_sec()
    while distance_traveled < distance:
        pub.publish(move_cmd)
        current_time = rospy.Time.now().to_sec()
        distance_traveled = speed * (current_time - start_time)
        rate.sleep()
    
    # Stop the robot after moving the required distance
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)

def rotate(pub, angular_speed, angle):
    # Create Twist message for rotation
    rotate_cmd = Twist()
    rotate_cmd.angular.z = angular_speed
    angle_rotated = 0.0
    rate = rospy.Rate(10)  # 10 Hz

    # Start time for angle calculation
    start_time = rospy.Time.now().to_sec()
    while angle_rotated < angle:
        pub.publish(rotate_cmd)
        current_time = rospy.Time.now().to_sec()
        angle_rotated = abs(angular_speed) * (current_time - start_time)
        rate.sleep()
    
    # Stop rotation after reaching desired angle
    rotate_cmd.angular.z = 0.0
    pub.publish(rotate_cmd)

def trace_pentagon():
    rospy.init_node('trace_pentagon', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Pentagon parameters
    speed = 0.2  # Linear speed (m/s)
    distance = 1.0  # Distance for each side in meters
    angular_speed = math.radians(30)  # Angular speed in radians/s (lower for precision)
    turn_angle = math.radians(72)  # Turn angle for pentagon (72 degrees)

    for _ in range(5):
        # Move forward for the length of each side
        move_forward(pub, speed, distance)
        rospy.sleep(1)  # Brief pause to ensure stability

        # Rotate 72 degrees for each corner of the pentagon
        rotate(pub, angular_speed, turn_angle)
        rospy.sleep(1)  # Brief pause before moving to the next side

if __name__ == "__main__":
    try:
        trace_pentagon()
    except rospy.ROSInterruptException:
        pass

