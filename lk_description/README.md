# lk\_description
This package contains the description of the robot used in the simulation in `lk_gazebo` and for the some of the physical locations for the controllers and sensor nodes in `lk_controls`.

## TODOs
- [ ] Add in the remaining actuators (Feb + 2 week)
  - [ ] Import Solidworks model (Feb + 3 weeks)
- [ ] Add in sensors (Feb + 1 week)
- [ ] Make sure package dependencies are correct (Feb + 1 week)

### Add in the remaining actuators (and other stuff)
Right now we only have actuators set up for the base of the robot, so it can move.
We don't have actuators set up for all the other stuff, so yeah, we need to add those.
The sizes and dimensions don't have to be perfect, we can get it up to spec later, but they should be close enough, and they'll be fixed in the next step.

The robot is described in URDF, which is explained [here](http://wiki.ros.org/urdf).
The body and wheels will all need to be inserted as `<link>`s, with `<joint>`s connecting them appropriately.
More info [here](http://wiki.ros.org/urdf/XML/link).
You'll also need to add the ROS plugin stuff so they can be controlled from ROS, using the stuff [here](http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros).

- [ ] Add actuators to the robot model (25 points/actuator)
- [ ] Make sure all dimensions are correct (50 points)

### Import Solidworks model
This is mainly for aethetics, but it'd be pretty correct if we got the exact shapes from Solidworks into Gazebo.
Convert the robot body and wheel Solidworks models into STL meshes, which can then be directly used in the URDF.
The position of the joints will still need to be set, but then we'll have the robot basically copied into gazebo.
The current description in the repository can act as a starting point.
More info [here](http://wiki.ros.org/urdf/XML/link) and [here](http://wiki.ros.org/Industrial/Tutorials/Create%20a%20URDF%20for%20an%20Industrial%20Robot).

- [ ] Add Solidworks models for wheels/body/deposition chain/deposition bucket/bucket ladder/bucket ladder bucket (25 points/part)

### Add in sensors
We also need to add the sensors to the URDF.
Basically the same procedure as above, each sensor probably has a matching ROS gazebo plugin to add its functionality into the simulation.
Like the actuators, this is divided into two steps, first just researching and adding all the sensors to the model, and secondly placing the sensors in the exact, correct location in the model.

- [ ] Add sensors to the robot model (25 points/sensor)
- [ ] Make sure all dimensions are correct (50 points)

### Check dependencies
In `package.xml`, there is a list of dependencies, which may or may not reflect what this package actually needs.
Try to figure out if that list is correct or not.

- [ ] Make sure dependencies for `lk_description` are correct (25 points)
