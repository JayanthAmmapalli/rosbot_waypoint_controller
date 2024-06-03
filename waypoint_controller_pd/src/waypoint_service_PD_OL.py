#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import math

class PDController:
    def __init__(self, target_x, target_y, target_theta):
        rospy.init_node('pd_controller_node', anonymous=True)

        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = target_theta

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.kp_linear = 1.0
        self.kd_linear = 0.1
        self.kp_angular = 2.0
        self.kd_angular = 0.1

        self.prev_error_linear = 0
        self.prev_error_angular = 0

        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)
        self.goal_reached = False

    def state_callback(self, data):
        try:
            index = data.name.index('rosbot')
            self.current_x = data.pose[index].position.x
            self.current_y = data.pose[index].position.y
            self.current_theta = self.quaternion_to_euler(data.pose[index].orientation)
        except ValueError:
            pass

    def quaternion_to_euler(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def compute_control(self):
        error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y

        distance_error = math.sqrt(error_x**2 + error_y**2)
        angle_to_target = math.atan2(error_y, error_x)
        heading_error = angle_to_target - self.current_theta

        # Normalize heading error to the range [-pi, pi]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        control_signal_linear = self.kp_linear * distance_error + self.kd_linear * (distance_error - self.prev_error_linear)
        control_signal_angular = self.kp_angular * heading_error + self.kd_angular * (heading_error - self.prev_error_angular)

        self.prev_error_linear = distance_error
        self.prev_error_angular = heading_error

        return control_signal_linear, control_signal_angular, distance_error, heading_error

    def run(self):
        while not rospy.is_shutdown():
            if self.goal_reached:
                cmd = Twist()
                self.pub.publish(cmd)
                rospy.loginfo("Goal reached.")
                break

            linear_vel, angular_vel, distance_error, heading_error = self.compute_control()
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.pub.publish(cmd)
            
            # Check if goal is reached
            if distance_error < 0.05 and abs(heading_error) < math.radians(5):
                self.goal_reached = True

            self.rate.sleep()

if __name__ == '__main__':
    target_x = float(input("Enter target x: "))
    target_y = float(input("Enter target y: "))
    target_theta = float(input("Enter target heading (in radians): "))
    controller = PDController(target_x, target_y, target_theta)
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass
