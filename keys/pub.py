#!/usr/bin/env python3
import sys
import select
import tty
import termios
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    key_pub = rospy.Publisher('keys', String, queue_size=1)
    rospy.init_node("keyboard_driver")
    rate = rospy.Rate(100)

    # Configure terminal to read single keypresses
    old_attr = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    print("Publishing keystrokes. Use 'w' to move forward, 'x' to move backward, 'a' to rotate left, 'd' to rotate right, and 's' to stop. Press Ctrl-C to exit...")

    try:
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                key_pub.publish(sys.stdin.read(1))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Reset terminal to original settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

