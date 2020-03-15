READ ME 

The merging control algorithm is used for the merging procedure of merging into a platoon. The merging algorithm works in the environment of Gazebo, from ROS. Gazebo is launched via the file gazebo_4_robots.launch. 

The python codes are the individual controllers for each agent. The codes are activated via the launch file platoon_merge_dynamic_4_robots_generationmu.launch

Step-by-step plan to start controller

1. $ roscore
2. $ roslaunch <PACKAGE_NAME> gazebo_4_robots.launch
3. $ roslaunch <PACKAGE_NAME> platoon_merge_dynamic_4_robots_generationmu.launch

Possible errors:
- Create ROS package via catkin_make <PACKAGENAME>
- Use the .msg file in the package folder 
- Errors in launching gazebo sometimes occur. Try again to launch.
- Some of the files in gazebo_4_robots.launch cannot be found due to the prescribed path. Check if all files are in the same locations in your computer.
- Files are not ready to be executed. Use the command $ chmod +x <FILENAME> in the correct directory

ENJOY!