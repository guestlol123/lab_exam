#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Define key mappings
key_mapping = {
    'f': [0, 0.5],   # Move forward with linear velocity
    'l': [0.5, 0],   # Rotate left with angular velocity
    'r': [-0.5, 0],  # Rotate right with angular velocity
    's': [0, 0]      # Stop
}

obstacle_detected = False

def laser_callback(msg):
    global obstacle_detected
    # Check if there's any obstacle within 1 meter in front of the robot
    front_ranges = msg.ranges[:30] + msg.ranges[-30:]  # Take a small range in front
    if any(distance < 0.7 for distance in front_ranges if distance > 0):
        obstacle_detected = True
    else:
        obstacle_detected = False

def keys_cb(msg, twist_pub):
    global obstacle_detected
    # Only process known keys
    if len(msg.data) == 0 or msg.data[0] not in key_mapping:
        return
    
    # Retrieve velocity from key_mapping
    angular_vel, linear_vel = key_mapping[msg.data[0]]
    
    # If an obstacle is detected and the command is to move forward, override to stop
    if obstacle_detected and linear_vel > 0:
        rospy.loginfo("Obstacle detected! Stopping forward movement.")
        linear_vel = 0  # Stop forward movement if obstacle is detected

    # Set up Twist message with the corresponding velocities
    twist = Twist()
    twist.angular.z = angular_vel
    twist.linear.x = linear_vel
    
    # Publish the Twist command
    twist_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('keys_to_twist_with_obstacle')
    twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, keys_cb, twist_pub)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    
    print("Listening for keystrokes: 'f' forward, 'r' rotate right, 'l' rotate left. Robot will stop if an obstacle is within 1 meter.")
    
    rospy.spin()

