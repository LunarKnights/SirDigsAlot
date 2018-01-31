# lk\_control
This package contains the configuration for all the controllers used in the robot.
Here we mean 'controller' in essentially the 'control theory' sense; each controller takes in a command from a ROS topic and does some math before sending a new command to the actual hardware through the interface set up in `lk\_base`.
This math can be a PID controller, or just directly copying the value into the hardware interface.
The corresponding ROS documentation is [here](http://wiki.ros.org/ros_control).

## TODOs
