# lk\_navigation
This package contains the configuration for the localization and movement planning systems.
Right now it actually doesn't have any of these things but we can fix that.
The `sirdigsalot` package will talk to the navigation stack located here using the `actionlib` C++ library, and the navigation stack will take in the sensor data and odometry data from `lk_base` or `lk_gazebo` and create a `cmd_vel` message that it'll send to `lk_control` to get the robot closer to its target.

## TODOs
- [ ] Research localization packages (Feb + 1 week)
  - [ ] Pick and configure different localization packages (Feb + 3 weeks)
    - [ ] Connect to move\_base node (Mar + 0 weeks)
        - [ ] Tune navigation stack (Mar + 2 weeks)
- [ ] Determine fiducials to use (Feb + 2 weeks)

### Research localization packages
We'll definitely be using a bunch of the different packages available from ROS for localization and mapping.
So we need someone to research a bunch of them, document what they all do, and put that here so it can be used for determining which one is the best fit for the robot.

- [ ] Research navigation-related ROS package (10 points/package)

### Pick and configure different localization packages
So after we have some of the different options researched, and narrowed it down to like 2 or 3 possible options, we need to actually add them to the system, configure them, and see how they perform.

Relevant materials:
- https://github.com/yunchih/ORB-SLAM2-GPU2016-final
- https://www.youtube.com/watch?v=p77hLLRfBGQ
- https://www.youtube.com/watch?v=BLdmNSWRuzU
- https://www.youtube.com/watch?v=EU6X1AYEksc
- http://openslam.org/gmapping
- https://www.mrpt.org/List_of_SLAM_algorithms
- https://github.com/googlecartographer/cartographer
- http://wiki.ros.org/octomap
- http://wiki.ros.org/rtabmap_ros

- [ ] Test out a localization package (25 points/package)

### Connect to move\_base node
After the localization and mapping stuff's taken care of, we just need to launch a `move_base` node and connect the topics so it'll work

Relevant materials:
- https://roscon.ros.org/2015/presentations/robot_localization.pdf

- [ ] Connect a move\_base node (25 points)

### Tune navigation node
After the move\_base node is correctly integrated into the system, tune it following the guide [here](http://kaiyuzheng.me/documents/navguide.pdf)

- [ ] Tune navigation (25 points)

### Determine fiducials to use
So we need to attach some kind of fiducial markings onto the deposition bucket to orient the robot relative to the bucket.
There seems to be two major kinds of fiducials (although there are probably more, go find them):
- ArUco, which is a 2D bar code
- (Fast) PiTag, which uses a set of circles

We need to figure out which one is better for our robot.
Both of these seem to be easyish to implement in ROS, ArUco is supported via the [aruco_detect](http://wiki.ros.org/aruco_detect) package, and PiTag is supported through [cob_fiducials](http://wiki.ros.org/cob_fiducials) (but the dependencies are an absolute mess so it might make sense to repackage this [un-ROSified version](https://github.com/mpetroff/pi-tag-detector) of it to avoid the dependency problems.

Here's some relevant documents:
- http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.721.540&rep=rep1&type=pdf
- http://www.uco.es/investiga/grupos/ava/sites/default/files/GarridoJurado2014.pdf

- [ ] Research fiducials (50-100 points)
