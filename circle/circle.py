#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_in_circle():
    # Initialize the ROS node
    rospy.init_node('circle_mover', anonymous=True)
    
    # Create a publisher to the /cmd_vel topic
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Set the rate to 10 Hz
    rate = rospy.Rate(10)
    
    # Define the Twist message for circular movement
    vel_msg = Twist()
    
    # Set a linear velocity in the x direction
    vel_msg.linear.x = 0.2  # Adjust this value for desired speed
    
    # Set an angular velocity for turning
    vel_msg.angular.z = 0.2  # Adjust this value to control the radius of the circle
    
    rospy.loginfo("Moving in a circle...")
    
    # Publish the message in a loop
    while not rospy.is_shutdown():
        # Publish the velocity
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_in_circle()
    except rospy.ROSInterruptException:
        pass

