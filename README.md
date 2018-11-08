# SirDigsAlot
The codebase for the Lunar Knights Mars Mining Robot.

This contains the code for autonomy for the robot, remote teleoperation, and some facilities for testing.

Currently the code is being divided into smaller packages to match practices in other ROS robots.
- sirdigsalot: High level automated control of the robot; this is where the logic should go. Also where the root launch files should go
- lk\_base: Interface to real hardware
- lk\_control: ROS controller stuff
- lk\_description: URDF description of the robot
- lk\_gazebo: Gazebo-related stuff; worlds and launch files to setup stuff
- lk\_navigation: Configuration for navigation stuff; exploration, SLAM, Kalman filters, etc
- lk\_teleop: Code to override the automated control and allow teleoperation

To run in gazebo:
```
USE\_GAZEBO=true roslaunch sirdigsalot bringup.launch
```

# How to contribute
Preliminary information on Git is available [here](https://help.github.com/articles/about-pull-requests/) and [here](https://git-scm.com/book/en/v1/Git-Branching-What-a-Branch-Is).
Also add yourself to the maintainers list in the `package.xml` file for that package, so that people working on that project down the line can bother you in the future.


# TODOs
These will be discussed in more detail in the READMEs in each package
- [ ] Improve robot modelling (lk\_gazebo, lk\_description)
- [ ] Add in remaining actuators for deposition system/camera system/whatever else (lk\_description)
- [ ] Implement controller lock node to allow cleaner transition between automated mode and teleop (lk\_control, new node)
- [ ] Make sure all package dependencies are correct (all packages)
- [ ] Readd e-stops/timeout checkers in various places (lk\_base, lk\_control)
- [ ] Reimplement teleoperation (lk\_teleop)
- [ ] Clean up launch files (all packages)
- [ ] Flesh out high level planning framework (sirdigsalot)
- [ ] Research and document the ROS navigation stack (lk\_navigation)
- [ ] Add/configure localization nodes (lk\_navigation)
- [ ] And/configure move\_base for robot navigation planning (lk\_navigation)
- [ ] Improve world simulation (lk\_gazebo)
- [ ] Add in hardware interfacing (CAN) code

# Setup instructions
These should probably get you a working copy of Sirdigsalot on your computer

## Install dependencies
```
// TODO: update
sudo apt-get install ros-kinetic-diff_drive_controller \
    ros-kinetic-controller-manager \
    ros-kinetic-fiducials \
    ros-kinetic-navigation
```
## Install into catkin\_ws/

```
# create a folder to put the code in
mkdir -p ~/catkin_ws

# copy the repository into ~/catkin_ws/src
git clone https://github.com/LunarKnights/SirDigsAlot src/

# set up the catkin workspace
cd ~/catkin_ws/src
catkin_init_workspace

# sync any Git submodules in the project
git submodule update --init --recursive

# build the project
source ~/catkin_ws/devel/setup.bash
cd ~/catkin_ws
catkin_make
```

# Useful references
http://wiki.ros.org/roscpp/Overview/Services
http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
http://wiki.ros.org/roscpp/Overview/Messages
http://gazebosim.org/tutorials?tut=ros_comm&cat=connect_ros
http://docs.ros.org/kinetic/api/gazebo_msgs/html/srv/JointRequest.html

