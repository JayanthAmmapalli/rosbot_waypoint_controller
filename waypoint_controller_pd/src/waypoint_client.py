#!/usr/bin/env python3

import rospy
from waypoint_controller_pd.srv import Waypoint, WaypointRequest

def call_waypoint_service():
    rospy.init_node('waypoint_client_node')
    rospy.wait_for_service('waypoint_service')

    try:
        waypoint_service = rospy.ServiceProxy('waypoint_service', Waypoint)
        waypoints = [
            (1.0, 1.0, 0.0),
            (2.0, 0.0, 90.0),
            (3.0, -1.0, 180.0)
        ]

        for x, y, psi in waypoints:
            request = WaypointRequest(x=x, y=y, psi=psi)
            response = waypoint_service(request)
            rospy.loginfo(f"Service response: success={response.success}, message={response.message}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    call_waypoint_service()
