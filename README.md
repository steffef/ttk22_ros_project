# ttk22_ros_project
This is the final project assignment 3 for the course TTK22 Cyber Physical Networked Vehicle Systems: Models, Algorithms, and Software Frameworks at NTNU. The code is not meant to be efficient or used in a practical setting, but rather an exercise in implementing a ROS module in an existing project. This module was made for the following repo: https://github.com/ntnu-arl/gbplanner_ros

## Setup
These instructions assume that ROS noetic is installed, and a ROS workspace has been set up. Then run:

```bash
cd <ros_ws>/src
git clone git@github.com:steffef/ttk22_ros_project.git
```

Then we need to clone necessary packages:
```bash
cd <ros_ws>
wstool init
wstool merge ./src/ttk22_ros_project/packages_ssh.rosinstall
wstool update
```

Install dependencies for ros
```bash
sudo apt-get install python3-rosdep
rosdep install --from-paths src --ignore-src -r -y
```

Finally we compile the code and source the catkin ws setup:

```bash
cd <ros_ws>
catkin config -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

The slam node can then be run with

```bash
roslaunch slam slam.launch
```