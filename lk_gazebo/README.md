# lk\_gazebo
This package contains the configuration and custom plugins for simulating the NASA RMC competition in gazebo.
Currently it just spawns the robot in the middle of an vast empty plain, but I think we can fix that!
Documentation on gazebo is available [here](http://gazebosim.org/tutorials).

## TODOs
- [ ] Randomize spawn positions (Feb + 2 weeks)
- [ ] Improve world (Feb + 2 weeks)
- [ ] Add collection bucket (Feb + 2 weeks)
- [ ] Add rocks at random locations (Feb + 2 weeks)
- [ ] Add scoring system (Mar + 2 weeks)
- [ ] Check package dependencies (Feb + 1 week)

### Randomize robot spawn position
In the actual competition the robot will be placed in a random orientation in one of two positions in the field.
So we need to simulate that in our simulation as well.

I'm pretty sure you just need to figure out the right change to this one line in `launch/simulation.launch`:
```
  <node name="spawn_rover" pkg="gazebo_ros" type="spawn_model" args="-unpause -param robot_description -urdf -z 1 -model sirdigsalot" />
```

Basically you should be able to add in some argument there to set the exact place the model gets spawned when it's run, so figure that out, and then find a way to make that position random.
Documentation [here](http://wiki.ros.org/simulator_gazebo/Tutorials/Gazebo_ROS_API#Spawn_Model) and [here](http://wiki.ros.org/roslaunch/XML#substitution_args).

- [ ] Randomize robot spawn position (25 points)

### Improve the world
So we need a more realistic test environment that will have the subtle features you'd expect to use in localization.
I'm thinking of using the surveying data from NASA's orbital probes, but they don't seem to have a resolution finer than 1 meter/pixel.
The sites for this data are [here](https://www.uahirise.org/dtm/) and [here](https://astrogeology.usgs.gov/search/details/Mars/GlobalSurveyor/MOLA/Mars_MGS_MOLA_DEM_mosaic_global_463m/cub), and also [here](https://webgis.wr.usgs.gov/pigwad/down/mars_hirise.htm).
This is probably fine, since we don't need that much precision just to get some dips and hills or whatever.
So yeah, try to crop out smaller sections of that that are a little bigger than the playing area, and get them to work inside gazebo (gazebo has some documentation on DEM stuff [here](http://gazebosim.org/tutorials?tut=dem)).
Try to make sure the height's fairly level near the starting area, or adjust it so it is, so the the robot's starting position doesn't need to be changed for different maps.

- [ ] Add hills and stuff to the map (25 points/map)
- [ ] Randomize map selection (50 points)

### Add collection bucket
Yeah, make a collection bucket and put it in about the right place in Gazebo.

- [ ] Make a collection bucket (25 points)

### Add rocks
Find some rock models, and spawn them in the world at random locations/orientations.
I'm pretty sure they're only allowed in the region between the excavation area and the starting area, so make sure they'll only spawn there.

- [ ] Add rocks (10 points)
- [ ] Randomize rock locations (25 points)

### Add scoring system
This is a bit more complicated than the other tasks listed so far, but it would allow fully automated testing.
Basically we need to make a Gazebo plugin that tracks when the deposition buckets dips under the terrain, and do some approximations to determine how much icy regolith it's gathering.
Then we use the time with the lip of the deposition system over the collection bucket to determine how much is deposited.
Excavation outside the right area should result in disqualification.

- [ ] Add scoring system (200 points)

### Check dependencies
In `package.xml`, there is a list of dependencies, which may or may not reflect what this package actually needs.
Try to figure out if that list is correct or not.

- [ ] Make sure dependencies for `lk_gazebo` are correct (25 points)
