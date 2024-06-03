#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

class TurnToAngle:
    def __init__(self, target_angle):
        rospy.init_node('turn_to_angle_node', anonymous=True)

        self.target_angle = target_angle  # Target angle in radians
        self.current_yaw = None

        self.angular_velocity = 7.33  # Maximum rotational velocity in rad/s
        self.tolerance = math.radians(5)  # 5 degree tolerance in radians

        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, data):
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(orientation_list)
        rospy.loginfo(f"Current Yaw: {self.current_yaw}")

    def run(self):
        rospy.loginfo("Waiting for /odom topic...")
        while self.current_yaw is None:
            rospy.sleep(0.1)
        rospy.loginfo("Starting turn...")

        while not rospy.is_shutdown():
            yaw_error = self.target_angle - self.current_yaw

            # Normalize yaw_error to the range [-pi, pi]
            yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

            if abs(yaw_error) < self.tolerance:
                rospy.loginfo("Target angle reached.")
                break

            turn_cmd = Twist()
            turn_cmd.angular.z = self.angular_velocity if yaw_error > 0 else -self.angular_velocity
            self.pub.publish(turn_cmd)
            rospy.loginfo(f"Publishing cmd_vel: {turn_cmd}")

            self.rate.sleep()

        # Stop the robot after turning
        stop_cmd = Twist()
        self.pub.publish(stop_cmd)
        rospy.loginfo("Turn completed.")

if __name__ == '__main__':
    target_angle_degrees = float(input("Enter target angle (in degrees): "))
    target_angle_radians = math.radians(target_angle_degrees)
    turn_to_angle = TurnToAngle(target_angle_radians)
    try:
        turn_to_angle.run()
    except rospy.ROSInterruptException:
        pass
