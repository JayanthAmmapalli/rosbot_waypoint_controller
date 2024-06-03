#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def turn_robot():
    # Initialize the ROS node
    rospy.init_node('turn_robot_node', anonymous=True)
    
    # Create a publisher to the '/cmd_vel' topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Set the rate at which to publish the messages
    rate = rospy.Rate(10)  # 10 Hz

    # Define the duration for the turn
    turn_duration = rospy.Duration(0.428)  # duration in seconds (adjust this based on the angular velocity)
    
    # Define the angular velocity
    angular_velocity = 7.33  # maximum rotational velocity in rad/s

    # Create a Twist message and set the angular velocity
    turn_cmd = Twist()
    turn_cmd.angular.z = angular_velocity  # Turn along the z-axis

    # Get the current time
    start_time = rospy.Time.now()
    
    rospy.loginfo("Starting turn...")
    # Publish the turn command for the specified duration
    for _ in range(int(10)):  # turn_duration in seconds * rate (10 Hz)
        pub.publish(turn_cmd)
        rospy.loginfo(f"Publishing cmd_vel: {turn_cmd}")
        rate.sleep()
    
    # Stop the robot after turning
    stop_cmd = Twist()
    pub.publish(stop_cmd)
    rospy.loginfo("Turn completed.")

if __name__ == '__main__':
    try:
        turn_robot()
    except rospy.ROSInterruptException:
        pass
