#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

def move_robot():
    # Initialize the ROS node
    rospy.init_node('move_robot_node', anonymous=True)
    
    # Create a publisher to the '/cmd_vel' topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Ensure the simulation clock is being used
    rospy.loginfo("Waiting for /clock topic...")
    rospy.wait_for_message('/clock', Clock)
    rospy.loginfo("/clock topic is active.")
    
    # Set the rate at which to publish the messages
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a Twist message and set the linear velocity
    move_cmd = Twist()
    move_cmd.linear.x = 0.5  # Move forward along the x-axis
    move_cmd.angular.z = 0.0  # No rotation
    
    # Get the current time
    start_time = rospy.Time.now()
    
    # Publish the message for 1 second
    while rospy.Time.now() - start_time < rospy.Duration(1.0):
        pub.publish(move_cmd)
        rate.sleep()
    
    # Stop the robot after 1 second
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Movement command published.")

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass
