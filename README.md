# KURAbot
Robot Operating System (ROS) based turtlebot2 project
(Updated: 6th January, 2016)

This project uses LiDAR (laser scanner) to localize the robot in a map using adaptive (or KLD-sampling) Monte Carlo localization (AMCL) approach. The robot moves to the fixed waypoint1 which is near the Chef's counter and waits for the visual command in the form of Augmented Reality Tags (ARTags). ARTags are being detected by Microsoft Kinect which detects the "id" of the tag along its depth from the robot. These visual commands dictates the robot to move to certain waypoints which depicts tables of the customers in the map. The robot takes the food to the ordered waypoint/table, rotates until it finds table "id" which is fixed for all tables to "ARTag 7", waits for 5 seconds and return to the Chef's counter for the next command.

Two launch files have to be run, one in the workstation and the other in turtlebot. They are present in the following location:

KURAbot/rbx2_tasks_workstation/rbx2_tasks/launch/deliverfood_workspace.launch
KURAbot/rbx2_tasks_workstation/rbx2_tasks/launch/deliverfood_turtlebot.launch

A python file has to be run in the workspace after the launch file is running. Following is the location of the python file:

KURAbot/rbx2_tasks_workstation/rbx2_tasks/nodes/delivering_food_10.py

The robot has to be placed in its initial position which is shown in the youtube video. The link of the video is as follows:

https://www.youtube.com/watch?v=Ak-OCw9Q0XY

