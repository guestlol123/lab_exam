#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math
import time

def move_in_pentagon():
    # Initialize the ROS node
    rospy.init_node('pentagon_move_node', anonymous=True)
    
    # Create a Publisher to send velocity commands
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    vel_msg = Twist()

    # Define the forward distance and turn angle for a pentagon
    forward_speed = 0.2  # meters per second
    turn_speed = 0.5     # radians per second
    side_length = 1.0    # distance for each side of the pentagon in meters
    angle = 72           # angle to turn at each vertex in degrees (pentagon)

    # Move and turn to form each side of the pentagon
    for _ in range(5):
        # Move forward for the length of one side
        vel_msg.linear.x = forward_speed
        vel_msg.angular.z = 0.0
        start_time = time.time()
        
        # Move forward for the distance of one side
        distance_moved = 0.0
        while distance_moved < side_length and not rospy.is_shutdown():
            velocity_publisher.publish(vel_msg)
            distance_moved = forward_speed * (time.time() - start_time)
            rate.sleep()

        # Stop the robot before turning
        vel_msg.linear.x = 0.0
        velocity_publisher.publish(vel_msg)
        time.sleep(1)

        # Turn by 72 degrees (1.256 radians) to create a pentagon
        vel_msg.angular.z = turn_speed
        target_turn_time = (angle * math.pi / 180) / turn_speed  # Calculate time for the turn
        start_turn_time = time.time()
        
        # Turn for the calculated duration
        while time.time() - start_turn_time < target_turn_time and not rospy.is_shutdown():
            velocity_publisher.publish(vel_msg)
            rate.sleep()

        # Stop the robot before the next side
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)
        time.sleep(1)

if __name__ == '__main__':
    try:
        move_in_pentagon()  # Move in a pentagon pattern
    except rospy.ROSInterruptException:
        pass
