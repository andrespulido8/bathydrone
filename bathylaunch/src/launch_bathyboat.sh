#!/bin/bash

rosrun xacro xacro ../simulation/boat_model/bathyboat_custom.urdf.xacro >  ../simulation/boat_model/bathyboat_custom.urdf
roslaunch vrx_gazebo sydneyregatta.launch urdf:=`pwd`/../simulation/boat_model/bathyboat_custom.urdf

