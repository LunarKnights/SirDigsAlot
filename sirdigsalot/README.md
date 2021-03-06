# sirdigsalot
This contains all the high level logic for the robot.
Currently it's basically empty, but it will consist of a lower level of logic that looks at sensor data and uses `actionlib` to direct `lk_navigation`, and an upper level that actually contains the logic.

This abstraction will let us avoid needing finite state machines, which are a very tedious to write and very prone to error.
Instead, we'll use a single thread that will run through the robot logic, using promises to encapsulate low level logic and communicate with the different systems as necessary.

_Promises_ are objects that promise to return a value eventually.
For examples, let's say we want to develop a function that causes the robot to turn 90 degrees.
We need this function to block until the robot has actually turned (or failed to turn) those 90 degrees.
To do this, we create a `promise` that promises to return the result of the turn attempt.
We attach some details of this promise to the ROS callbacks used to collect data from the other systems, to detect when the turn was successfully completed (or failed).
When this condition is detected within one of the callbacks, it pushes the result into the promise, triggering out main thread to unblock and continue on in its logic.

## TODOs
- [ ] Implement all low level abstractions (Mar + 0 weeks)
- [ ] Design high level logic (Mar + 0 weeks)
- [ ] Implement low level framework (Feb + 2 weeks)
  - [ ] Implement high level logic (Mar + 1 week)
    - [ ] Write tests for high level logic (Mar + 3 weeks)

### Design high level logic
We need to write some pseudocode to efficiently do the autonomous mining.
This code should be very robust, with as many edge cases accounted for as possible.
Unlike many of the other tasks listed here, I think this should be a group effort.

- [ ] Design high level logic (100 points)

### Implement low level framework
I'll be writing a fairly simple API for the low level logic stuff.
These don't need to be too performant, because the logic will be done on the order of milliseconds to seconds.
That includes:

- [ ] `move_base` for robot planning
- [ ] controllers for actuation
- [ ] fiducial tracker for locating the deposition bucket beacons
- [ ] camera feed for (potentially) measuring the amount of collected regolith

Relevant documentation:
- https://answers.ros.org/question/12276/is-there-a-c-api-for-a-dynamic-reconfigure-client/
- http://docs.ros.org/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#a16a367c503a355b3872f046cdf914c65
- http://wiki.ros.org/move_base?distro=lunar#move_base-1
- http://en.cppreference.com/w/cpp/thread/condition_variable
- http://www.ros.org/reps/rep-0105.html
- https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf (might be useful for averaging fiducial locations)

- [ ] Implement `move_base` wrapper (50 points)
- [ ] Implement controller wrapper (50 points)
- [ ] Implement fiducial tracker wrapper (50 points)
- [ ] Implement camera feed wrapper (50 points)

### Implement high level logic
Use the low level API to implement the high level pseudocode developed before.

- [ ] Implement high level logic (200 points)

### Write tests for high level logic
We need to figure out how to do full scale integration tests, and preferably find a way to automate them as much as possible.

- [ ] Write tests for high level logic (100 points)
