# lk\_base
This is the package that connects the control inputs from the nodes in lk\_control to actual hardware (or gazebo).
The lk\_control will connect to the `ControllerManager` created in this package, and spawn controllers to connect ROS control topics (like cmd\_vel) to the hardware.
The corresponding ROS documentation is [here](http://wiki.ros.org/ros_control).



### Connect the actuators to the motor controllers
Right now all this package does is provide a place for controllers to put their commands and get their sensor data from.
Just to recap the ROS control documentation, when a controller wants to tell a joint on the robot to do something, it asks the controller manager to find the joint in the hardware interface based on where it was registered, and then writes the command it wants to send to the joint there.
Similarly, when a controller wants to get the latest sensor data, it'll ask the controller manager, who'll find where it was registered, and then the controller can read the values it wants from there.
Technically, it's a little more complicated than that, but this is a good enough explanation.

So, after the previous step is done, the controllers will be periodically writing data to some floating-point values in `LKHardware`, and then reading some values from other floating-point numbers in `LKHardware`.
Now we need to make sure those numbers are meaningful values captured from the real world.


### Add timeouts/emergency stops
For added safety, we need to add some code to automatically disable all the motors whenever something that's potentially bad happens.
This should include:

- loss of connection during teleoperation
- no messages from the navigation node (maybe it died or something?)
- command from a ROS topic that acts as a software emergency stop (and also don't forget to connect that to something in teleoperation mode)

So we need to figure out exactly what kinds of safety measures should be done, and then actually do that, and then write some integration tests to make sure it actually works.


### Add all sensors
We need to compile a list of all the sensors on the robot and their positions on it too.
Each sensor will probably have a ROS node that will communicate with it and publish their data for the sensor fusion stuff to use.

```
Sensor list here
```

### Check dependencies
In `package.xml`, there is a list of dependencies, which may or may not reflect what this package actually needs.
Try to figure out if that list is correct or not.

- [ ] Make sure dependencies for `lk_base` are correct (25 points)
