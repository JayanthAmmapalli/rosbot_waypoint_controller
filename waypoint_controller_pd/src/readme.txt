Instructions to start rosbot simulation in gazebo and using PD waypoint controller to move the rosbot.

1. Launch rosbot in gazebo using this command(if the rosobot gazebo packages are not installed use this link to download required files https://github.com/husarion/rosbot_ros)
    >> roslaunch rosbot_bringup rosbot_tutorial.launch
2. Run waypoint service that take waypoints as input and run PD controller
    >> rosrun waypoint_controller_pd waypoint_service_pd.py
3. Run waypoint client that publishes waypoints in sequence as given in this file:
    >> rosrun waypoint_controller_pd waypoint_client.py 