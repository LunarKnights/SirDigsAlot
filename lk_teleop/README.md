# lk\_teleop
This node is run remotely, allowing for teleoperation of the robot.
It uses an Xbox 360 controller for input, and sends its commands to lk\_control.

## TODOs
- [ ] Find new controller mapping (Feb + 1 week)
  - [ ] Implement controller (Feb + 3 weeks)

### Find new controller mapping
Since the new design is a bit more complicated than the previous year's robot, we need a new controller mapping.
Also it'd be nice to get a mapping for the keyboard too, for testing without the controller.

- [ ] Make a new controller mapping (10 points)
- [ ] Make a new keyboard mapping (10 points)

### Implement the new controller
So we need to remap the keys and also connect the controls with the new ROS topics that the robot will be exposed in `lk_controls`.

- [ ] Implement teleoperation for controller (25 points)
- [ ] Implement new teleoperation for keyboard (25 points)
