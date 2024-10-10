
Code for meeting requirements of robotics studio 41068 assignment: sprint 3

Launch Files:

SLO3-2mapping.launch:
	launches all programs required to create a map of the smart factory world. use the teleop keyboard to move the platform around and generate the map using nav2
	
SLO3-5pathPlanning.launch:
	launches programs required to navigate the turtlebot around the environment using nav2goal button in rVis, make sure the use the localisation button to tell the robot where it is before setting goals

SLO3-6objectExclude.launch:
	launches the robot in the gazebo world where you can manually move the robot and the cylinder around. When code detects the cylinder it will stop looking and generate a map with the cylinder overlaid.

dependancies:
	smart_factory package (https://github.com/AbhinashTiwary0509/RS1)

Date: 10/10/2024
Assignment: Robotics Studio 1 Sprint 3
Author: Charles Fawcett, charles.fawcett@student.uts.edu.au


