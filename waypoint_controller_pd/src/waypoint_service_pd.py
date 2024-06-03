#!/usr/bin/env python3

# PD controller for rosbot in gazebo simulator
# Takes current position form Rosbot name in /odom topic
# Publishes Twist messages to /cmd_vel topic which is then picked up by rosbot

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from waypoint_controller_pd.srv import Waypoint, WaypointResponse
import math

class PDController:
    def __init__(self):
        rospy.init_node('pd_controller_node', anonymous=True)

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.kp_linear = 1.0
        self.kd_linear = 0.1
        self.kp_angular = 2.0
        self.kd_angular = 0.2

        self.max_linear_velocity = 1.0  # Maximum translational velocity in m/s
        self.max_angular_velocity = 7.33  # Maximum rotational velocity in rad/s

        self.prev_error_linear = 0
        self.prev_error_angular = 0

        self.position_tolerance = 0.1  # Position tolerance in meters
        self.yaw_tolerance = 0.1  # Yaw tolerance in radians

        self.prev_time = rospy.Time.now()  # Initialize prev_time

        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.state_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)
        self.service = rospy.Service('waypoint_service', Waypoint, self.handle_waypoint_service)

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

    def compute_position_control(self, target_x, target_y):
        error_x = target_x - self.current_x
        error_y = target_y - self.current_y

        distance_error = math.sqrt(error_x**2 + error_y**2)
        angle_to_target = math.atan2(error_y, error_x)
        heading_error = angle_to_target - self.current_theta

        # Normalize heading error to the range [-pi, pi]
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        control_signal_linear = self.kp_linear * distance_error + self.kd_linear * (distance_error - self.prev_error_linear)
        control_signal_angular = self.kp_angular * heading_error + self.kd_angular * (heading_error - self.prev_error_angular)

        self.prev_error_linear = distance_error
        self.prev_error_angular = heading_error

        # Clamp the velocities to the maximum values
        control_signal_linear = max(min(control_signal_linear, self.max_linear_velocity), -self.max_linear_velocity)
        control_signal_angular = max(min(control_signal_angular, self.max_angular_velocity), -self.max_angular_velocity)

        return control_signal_linear, control_signal_angular, distance_error, heading_error

    def compute_yaw_control(self, target_theta):
        yaw_error = target_theta - self.current_theta

        # Normalize yaw_error to the range [-pi, pi]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        # PD Control
        p_term = self.kp_angular * yaw_error
        d_term = self.kd_angular * (yaw_error - self.prev_error_angular) / dt if dt > 0 else 0.0

        angular_velocity = p_term + d_term
        angular_velocity = max(min(angular_velocity, self.max_angular_velocity), -self.max_angular_velocity)

        self.prev_error_angular = yaw_error
        self.prev_time = current_time

        return angular_velocity, abs(yaw_error)

    def handle_waypoint_service(self, req):
        rospy.loginfo("Received waypoint service call")

        target_x = req.x
        target_y = req.y
        target_theta = math.radians(req.psi)  # Convert degrees to radians

        rospy.loginfo(f"Navigating to: x={target_x}, y={target_y}, theta={req.psi} degrees")

        # Navigate to the target position
        self.position_goal_reached = False
        while not rospy.is_shutdown() and not self.position_goal_reached:
            linear_vel, angular_vel, distance_error, heading_error = self.compute_position_control(target_x, target_y)
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.pub.publish(cmd)

            if distance_error < self.position_tolerance and abs(heading_error) < math.radians(5):
                self.position_goal_reached = True
                rospy.loginfo("Position goal reached.")

            self.rate.sleep()

        # Align to the target yaw
        self.yaw_goal_reached = False
        while not rospy.is_shutdown() and not self.yaw_goal_reached:
            angular_vel, yaw_error = self.compute_yaw_control(target_theta)
            cmd = Twist()
            cmd.angular.z = angular_vel
            self.pub.publish(cmd)

            if yaw_error < self.yaw_tolerance:
                self.yaw_goal_reached = True
                rospy.loginfo("Yaw goal reached.")

            self.rate.sleep()

        # Stop the robot after reaching the waypoint
        cmd = Twist()
        self.pub.publish(cmd)
        rospy.loginfo("Waypoint reached.")

        return WaypointResponse(success=True, message="Waypoint navigation completed.")

if __name__ == '__main__':
    controller = PDController()
    rospy.spin()
