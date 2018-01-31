# lk\_control
This package contains the configuration for all the controllers used in the robot.
Here we mean 'controller' in essentially the 'control theory' sense; each controller takes in a command from a ROS topic and does some math before sending a new command to the actual hardware through the interface set up in `lk_base`.
This math can be a PID controller, or just directly copying the value into the hardware interface.
The corresponding ROS documentation is [here](http://wiki.ros.org/ros_control).

## TODOs
- [ ] Write \\cmd\_vel multiplexer (Feb + 2 weeks)
- [ ] Add controllers for all the joints in the robot (Feb + 2 weeks)
- [ ] Tweak controllers for all the joints in the robot (March + 2 weeks)
- [ ] Make sure package dependencies are correct (Feb + 1 week)

### Write \\cmd\_vel multiplexer
So `\cmd_vel` is a standardish ROS topic used for communicating the direction the robot should head in.
The navigation node, or the user in teleoperation mode, will send commands to `\cmd_vel`, and the controllers set up here will figure out the appropriate commands to send to the wheel actuators to get the robot moving in that direction.

The problem here is that the controller has no way to tell the difference between a `\cmd_vel` message sent from teleoperation versus one from navigation.
If the controllers start receiving messages from both of them at the same time, how does it tell which one it should listen to?

This is why we need a multiplexer node.
This node acts like a switch, deciding whether to send messages from the navigation stack to the controllers, or to send the messages from the teleoperation node instead.
It subscribes to two ROS topics that the navigation stack and the teleoperation node will respectively subscribe to, and it'll do some simple logic to determine which messages it will forward to the `\cmd_vel` topic used by the controller.
It needs to automatically switch from receiving from the navigation stack to receiving from the teleoperation node, and preferably there should be a way to set which source to use.

There are a lot of command messages that will be used, for every single actuator on the robot, so this node will need to do this multiplexing for each ROS topic that's used for commanding the robot.

- [ ] Implement command multiplexer (50 points)
- [ ] Configure launch scripts to use multiplexer (25 points)
- [ ] Write tests for multiplexer (100 points)

### Add controllers for all the joints in the robot
