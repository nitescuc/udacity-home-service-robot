# Installation

In project's workspace as provided by Udacity (VM) required python package rospkg is missing. It is mandatory to run ```pip install rospkg``` prior to any test or turtlebot world won't run in gazebo.

# Observations

add_markers initial behavior require to display the marker at pickup, hide it for 5 seconds then display it again at drop point. This behavior doesn't exist anymore in the submited version since this node is integrated with pick_objects.

# Packages used

## Localization

Localization is implemented here using AMCL ros package: http://wiki.ros.org/amcl

The algorithm here is Adaptive Monte Carlo.

## Mapping

Mapping is done with gmapping ros package using laser sensor and robot pose: http://wiki.ros.org/gmapping

In a previous step we drive the robot into the environment to generate a map file (pgm) that will be used afterwards for localization and navigation. 

## Navigation

Navigation is done with move_base ros package: http://wiki.ros.org/move_base

This package is able to compute a path using the map and drive the robot to a goal. Also there are messages allowing to monitor the state of the robot (position and status reaching the goal)
