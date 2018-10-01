# ROS navigation implementation using move_base package

## Global path planner
The file **Astar_global_planner.cpp** contains the derived class **Astar** which implementents the base class **BaseGlobalPlanner**. 

## Local path planner 
The file **DW_local_planner.cpp** contains the derived class **DynamicWindow** which implementents the base class **BaseLocalPlanner**.

Both classes are exported as plugins and included in the move_base navigation stack.

## Installing
To install this project, you have to have ROS installed alongside the following ROS packages:
- stage_ros
- gmapping
- rqt_gui
- rviz
- move_base

You also have to have a Workspace set up, then you can simple clone this project into the /src folder of your Workspace.

## Building
Simply run catkin_make in your workspace. Then source the setup file in the /devel folder.

## Running
You can run this project by typing ` roslaunch pathplanner CustomCompletePlanner.launch `

Developed by Muhamed Delalić

This project is licensed under the terms of the MIT license.
