#!/bin/bash
cd ../../..
catkin_make
cd src/cse_190_pa3/scripts
roslaunch cse_190_assi_3 solution_python.launch
