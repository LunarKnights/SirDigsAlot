# SirDigsAlot
The codebase for the Lunar Knights Mars Mining Robot.

This contains the code for autonomy for the robot, remote teleoperation, and some facilities for testing.

Currently the code is being divided into smaller packages to match practices in other ROS robots.
lk\_base: Interface to real hardware
lk\_control: ROS controller stuff
lk\_description: URDF description of the robot
lk\_gazebo: Gazebo-related stuff; worlds and launch files to setup stuff
lk\_navigation: Configuration for navigation stuff; exploration, SLAM, Kalman filters, etc
sirdigsalot: High level automated control of the robot; this is where the logic should go. Also where the root launch files should go
lk\_teleop: Code to override the automated control and allow teleoperation

To run in gazebo:
USE\_GAZEBO=true roslaunch sirdigsalot bringup.launch
  
# Install dependencies
```

sudo apt-get install ros-kinetic-diff_drive_controller

sudo apt-get install ros-kinetic-controller_manager
//Needs update
```
# How to install

```
# create a folder to put the code in
mkdir -p ~/catkin_ws

# copy the repository into ~/catkin_ws/src
git clone https://github.com/LunarKnights/SirDigsAlot src/

# set up the catkin workspace
cd ~/catkin_ws/src
catkin_init_workspace

# build the project
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make
```

# Developer notes
The code should be formatted against the ROS standard so that's everything's nice and uniform.
The easiest way to do this is using `clang-tidy` with this guy's [formatting rules](https://github.com/davetcoleman/roscpp_code_format)

This is how to install `clang-tidy` and that guy's rules:
```
sudo apt-get install -y clang-format-3.8
cd ~
git clone https://github.com/davetcoleman/roscpp_code_format
ln -s ~/roscpp_code_format/.clang-format ~/catkin_ws/.clang-format
```

and then run this from the `src/` directory to restyle any files you've changed:
```
find . -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-3.8 -i -style=file $1
```


## Order to read files
These are currently fully commented:
- lk\_rover/src/lk\_rover\_node.cpp
- lk\_rover/src/lk\_rover\_controller.cpp
- lk\_rover/src/lk\_rover.cpp
- lk\_rover/src/lk\_rover\_hw.cpp
- lk\_rover/src/gazebo\_hw.cpp
- arduino\_stuff/arduino\_motor\_controller/arduino\_motor\_controller.ino

## Useful references
http://wiki.ros.org/roscpp/Overview/Services
http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
http://wiki.ros.org/roscpp/Overview/Messages
http://gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros
http://docs.ros.org/kinetic/api/gazebo_msgs/html/srv/JointRequest.html

