#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

class TurnToAnglePD:
    def __init__(self, target_angle):
        rospy.init_node('turn_to_angle_pd_node', anonymous=True)

        self.target_angle = target_angle  # Target angle in radians
        self.current_yaw = None

        self.kp = 2.0  # Proportional gain
        self.kd = 0.1  # Derivative gain
        self.max_angular_velocity = 7.33  # Maximum rotational velocity in rad/s
        self.tolerance = math.radians(5)  # 5 degree tolerance in radians

        self.prev_yaw_error = 0
        self.prev_time = rospy.Time.now()

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

            current_time = rospy.Time.now()
            dt = (current_time - self.prev_time).to_sec()

            # PD Control
            p_term = self.kp * yaw_error
            d_term = self.kd * (yaw_error - self.prev_yaw_error) / dt if dt > 0 else 0.0

            angular_velocity = p_term + d_term
            angular_velocity = max(min(angular_velocity, self.max_angular_velocity), -self.max_angular_velocity)

            turn_cmd = Twist()
            turn_cmd.angular.z = angular_velocity
            self.pub.publish(turn_cmd)
            rospy.loginfo(f"Publishing cmd_vel: {turn_cmd}")

            self.prev_yaw_error = yaw_error
            self.prev_time = current_time

            self.rate.sleep()

        # Stop the robot after turning
        stop_cmd = Twist()
        self.pub.publish(stop_cmd)
        rospy.loginfo("Turn completed.")

if __name__ == '__main__':
    target_angle_degrees = float(input("Enter target angle (in degrees): "))
    target_angle_radians = math.radians(target_angle_degrees)
    turn_to_angle_pd = TurnToAnglePD(target_angle_radians)
    try:
        turn_to_angle_pd.run()
    except rospy.ROSInterruptException:
        pass
