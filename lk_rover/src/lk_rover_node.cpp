/// This is the main file for the lk_rover node
/// This is the main logic for the system
/// Below we have a lot of #includes for:

/// Standard C++ features that we need, like vectors and atomic variables
#include <atomic>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

/// The basic #includes you need when you're writing ROS code;
/// these provide functions that you need to talk to other ROS nodes
#include "ros/ros.h"
#include "ros/console.h"

/// Here we include std_msgs/Bool.h because it provides the stuff for a Bool message,
/// which we use to trigger teleoperation mode
#include "std_msgs/Bool.h"

/// These are for setting up the controllers for the motors
#include "controller_manager/controller_manager.h"
#include "controller_manager_msgs/SwitchController.h"

/// These are for using the other files
#include "lk_rover/lk_controller.h"
#include "lk_rover/lk_rover.h"
#include "lk_rover/lk_rover_hw.h"

/// This is a weird hack I'm using right now
/// This disables the code for the gazebo stuff so the code will compile without
/// needing gazebo installed
/// I should probably fix this at some point..
#undef USE_GAZEBO

/// #ifdefs, if you haven't used them before, are kind of like meta-if statements
/// If the USE_GAZEBO is defined, then the code between the #ifdef and the #endif
/// will be compiled
/// Contrawise, if USE_GAZEBO is not defined, then the code won't be compiled
/// This is so that eventually, when I figure out how to set up USE_GAZEBO correctly,
/// the build tools automatically define USE_GAZEBO when it detects gazebo on the
/// system, so your version of the code will have gazebo support if your system
/// has gazebo
#if USE_GAZEBO

#include "gazebo_msgs/SpawnModel.h"
#include "lk_rover/gazebo_hw.h"

bool SpawnModel(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);

#endif

/// Here we set up the variable we use for checking whether the robot's still connected
/// with the teleop device
/// Basically, the teleop device will periodically send a message to the robot,
/// and the function that processes that message will set this to true when it
/// receives the message. When the main loop of the program runs, it checks to
/// see if this is set to true, and then resets it to false to detect if another
/// heartbeat happens before the next loop of the program. If we don't detect a
/// heartbeat for some specified number of loops, we stop the robot
std::atomic<bool> receivedHeartbeat(false);
void teleopHeartCb(const std_msgs::Bool &b) {
  receivedHeartbeat.store(true);
};

int main(int argc, char** argv) {
  /// At the top of any ROS program, you'll probably need to call ros::init
  ros::init(argc, argv, "lk_rover_base");

  /// ... and then this stuff to get access to ROS topic stuff
  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  /// This shared_ptr will store the hardware interface we'll use to actually
  /// read from the motor encoders and set the motor speeds
  /// It's set up as a pointer so that it'd be possible to swap out the gazebo
  /// and actual implementations of the hardware stuff without changing much code
  std::shared_ptr<LKHW> hw;
#if USE_GAZEBO

  /// If the code is compiled with gazebo support, it'll check for a ROS parameter
  /// telling it whether to use gazebo
  bool useGazebo = false;
  nhPrivate.param<bool>("use_gazebo", useGazebo, false);

  if (useGazebo) {
    /// If it is using gazebo, then it waits for gazebo to initialize...
    ROS_INFO("waiting for gazebo...");
    int timeout_count = 5;
    int timeout_time = 5;
    while (timeout_count > 0) {
      if (ros::service::waitForService("/gazebo/spawn_urdf_model", timeout_time)) {
        break;
      }
      timeout_count--;
      ROS_INFO("/gazebo/spawn_urdf_model connection timed out, retry %d", 5-timeout_count);
    }
    if (timeout_count <= 0) {
      ROS_ERROR("unable to connect to gazebo");
      return -5;
    }

    /// ...spawns the robot model into gazebo...
    ROS_INFO("spawning model...");
    // spawn the robot model in gazebo
    if (!SpawnModel(nh, nhPrivate)) {
      ROS_ERROR("unable to spawn model");
      return -1;
    }

    /// ... and then stores the gazebo motor object in the shared_ptr
    auto gazeboHW = std::make_shared<GazeboHW>();
    if(!gazeboHW->init(nh)) {
      ROS_ERROR("gazebo hw init failed");
      return -8;
    }
    hw = gazeboHW;
  } else {
#endif
    /// Otherwise just make the normal hardware object and put it in the pointer
    hw = std::make_shared<LKRoverHW>(nh);
    // wait for the rosserial link to connect
    dynamic_cast<LKRoverHW*>(hw.get())->waitForSerial();
#if USE_GAZEBO
  }
#endif

  /// Now we'll set up the configuration for the custom PID 
  /// These are for ensuring the two motors for the ladder
  /// and the bucket remain basically in the same position,
  /// as well as for using the calibration values for the
  /// string potentiometers
  ActuatorConfigs dumpConfigs = {0}, ladderConfigs = {0};

  /// Basically copy data from ROS, and fail if anything's missing
  bool configFail = false;
  if (!nh.getParam("dump/left/min", dumpConfigs.left.min)) {
    ROS_ERROR("unable to find dump left min");
    configFail = true;
  }
  if (!nh.getParam("dump/left/max", dumpConfigs.left.max)) {
    ROS_ERROR("unable to find dump left max");
    configFail = true;
  }
  if (!nh.getParam("dump/left/gain", dumpConfigs.left.gain)) {
    ROS_ERROR("unable to find dump left gain");
    configFail = true;
  }
  if (!nh.getParam("dump/right/min", dumpConfigs.right.min)) {
    ROS_ERROR("unable to find dump right min");
    configFail = true;
  }
  if (!nh.getParam("dump/right/max", dumpConfigs.right.max)) {
    ROS_ERROR("unable to find dump left max");
    configFail = true;
  }
  if (!nh.getParam("dump/right/gain", dumpConfigs.right.gain)) {
    ROS_ERROR("unable to find dump right gain");
    configFail = true;
  }
  if (!nh.getParam("dump/diff_gain", dumpConfigs.diffGain)) {
    ROS_ERROR("unable to find dump diff gain");
    configFail = true;
  }
  if (!nh.getParam("dump/length", dumpConfigs.length)) {
    ROS_ERROR("unable to find dump diff length");
    configFail = true;
  }
  ROS_INFO("dump params: %lf %lf %lf %lf %lf %lf %lf", 
    dumpConfigs.left.min, dumpConfigs.left.max, dumpConfigs.left.gain,
    dumpConfigs.right.min, dumpConfigs.right.max, dumpConfigs.right.gain,
    dumpConfigs.diffGain);

  if (!nh.getParam("ladder/left/min", ladderConfigs.left.min)) {
    ROS_ERROR("unable to find ladder left min");
    configFail = true;
  }
  if (!nh.getParam("ladder/left/max", ladderConfigs.left.max)) {
    ROS_ERROR("unable to find ladder left max");
    configFail = true;
  }
  if (!nh.getParam("ladder/left/gain", ladderConfigs.left.gain)) {
    ROS_ERROR("unable to find ladder left gain");
    configFail = true;
  }
  if (!nh.getParam("ladder/right/min", ladderConfigs.right.min)) {
    ROS_ERROR("unable to find ladder right min");
    configFail = true;
  }
  if (!nh.getParam("ladder/right/max", ladderConfigs.right.max)) {
    ROS_ERROR("unable to find ladder left max");
    configFail = true;
  }
  if (!nh.getParam("ladder/right/gain", ladderConfigs.right.gain)) {
    ROS_ERROR("unable to find ladder right gain");
    configFail = true;
  }
  if (!nh.getParam("ladder/diff_gain", ladderConfigs.diffGain)) {
    ROS_ERROR("unable to find ladder diff gain");
    configFail = true;
  }
  if (!nh.getParam("ladder/length", ladderConfigs.length)) {
    ROS_ERROR("unable to find ladder diff length");
    configFail = true;
  }
  ROS_INFO("ladder params: %lf %lf %lf %lf %lf %lf %lf", 
    ladderConfigs.left.min, ladderConfigs.left.max, ladderConfigs.left.gain,
    ladderConfigs.right.min, ladderConfigs.right.max, ladderConfigs.right.gain,
    ladderConfigs.diffGain);

  if (configFail) exit(-1);

  /// Make the LKRover object
  /// This is what ROS's ros_controllers library talks to to do all
  /// the control system stuff
  /// Basically all it does it talk to the HW object it made earlier to
  /// get the right data
  LKRover robot(hw, dumpConfigs, ladderConfigs);
  /// Start up ROS's controller manager
  /// This controls the controllers individually
  controller_manager::ControllerManager cm(&robot, nh);

  /// The controller manager gets the parameters and information about its features
  /// from rosparams, which in this case it get from config/control.yaml
  cm.loadController("lk_velocity_controller");
  cm.loadController("lk_dump_controller");
  cm.loadController("lk_ladder_controller");
  cm.loadController("lk_spin_controller");
  cm.loadController("lk_flap_controller");

  /// Here's the variables we need for controlling the program overall,
  /// basically just a couple different flags that we'll check in loops
  /// to make sure the robot should still be running

  /// teleopMode is obviously for determining if the robot is in teleop mode
  bool teleopMode = false;

  /// running is used to stop the control thread when ROS determines it is
  /// being shut down
  /// killMotors is used to temporarily stop the motors in case of signal loss
  /// when in teleop mode
  std::atomic<bool> running(true);
  std::atomic<bool> killMotors(false);

  /// When in teleop mode, this node will periodically receive a message on this
  /// topic
  auto heartSub = nh.subscribe("heartbeat", 1, teleopHeartCb);

  /// Here it starts another thread that will run in parallel with the rest of
  /// this code
  /// This is because the controllers need to be running basically constantly,
  /// so it makes sense to make it run in a different thread so it won't be
  /// blocked on what's happening in this thread

  /// The syntax here's a little bit complicated, it's passing a function as
  /// an argument into the constructor for std::thread, which tells std::thread
  /// to run that function when the thread is started

  /// The [&] part of it says that the function (btw it's called a lambda expression
  /// in C++) says that the function can use variables by reference. This is in
  /// contrast to using variables by copy, where the function gets copies of any
  /// variables in the environment that it uses (which is signified by [=] instead).
  /// This function needs to use the variables by reference so that both the current
  /// thread and the thread being spawned can communicate with each other by
  /// changing those variables
  ///
  /// For example, the running variable is used inside to determine if the control
  /// thread needs to keep running. If the function was capture by value instead,
  /// then modifying the running variable out in the main thread wouldn't change
  /// the running variable in the control thread, so we wouldn't be able to stop
  /// the control thread using a variable like this
  std::thread controlThread([&]() {
    auto r = ros::Rate(50);
    auto curTime = ros::Time::now();
    /// Basically this just loops continuously
    while (running) {
      /// Sleeping so that it runs at ~50 Hz
      r.sleep();
      /// Getting data from the encoders
      robot.read();
      /// And then, based on the various flags, either
      if (!killMotors) {
        /// (1) run the controller
        robot.unkillMotors();
        cm.update(curTime, r.cycleTime());
      } else {
        /// or (2) stop the motors
        robot.killMotors();
      }
      /// and then write the new PWM values to the hardware interface
      robot.write();
      // ROS_INFO("control loop");
    }
  });

  /// Now we start all the different controllers
  auto toStart = std::vector<std::string>{
    "lk_velocity_controller",
    "lk_dump_controller",
    "lk_ladder_controller",
    "lk_spin_controller",
    "lk_flap_controller",
  };
  auto toStop = std::vector<std::string>{};
  cm.switchController(
      toStart,
      toStop,
      2); // STRICT

  /// Here we create the LKController object
  /// This is where all the finite state machine logic and stuff would've happened
  /// if we finished autonomy in time
  auto master = LKController(nh, nhPrivate);

  /// Anyways, now's the main function loop that this thread will run until the
  /// robot's killed
  /// This runs at 100 Hz
  auto r = ros::Rate(100);
  /// And this stuff configures how many lost heartbeats are necessary
  /// for the robot to automatically kill the motors
  const int kTimeoutTime = 20;
  auto teleopTimeout = kTimeoutTime;
  while (ros::ok()) {
    r.sleep();
    /// If it's in not telop mode, it'll allow the LKController object to
    /// to do stuff
    if (!teleopMode) {
      master.doStuff();
    } else {
      /// Otherwise, it'll do the teleop logic, which is to
      /// check for a heartbeat
      if (!receivedHeartbeat) {
        /// decrement the timeout if it's not detected
        --teleopTimeout;
      } else {
        /// reset the timeout count if it is
        teleopTimeout = kTimeoutTime;
        killMotors.store(false);
      }
      /// And if the timeout reaches zero kill the motors
      if (teleopTimeout == 0) {
        ROS_WARN("teleop connection loss detected; killing motors");
        killMotors.store(true);
      }
      /// The teleoperation node directly talks to the motor drivers, so
      /// there's no need to mess with that stuff here
      /// This is all just to prevent the robot from going rogue and killing
      /// all humans
      /// or whatever
    }
    /// Enter teleoperation mode if a heartbeat's detected
    if (receivedHeartbeat) {
      teleopMode = true;
    }
    /// Then reset the variable so it can be set again if another heartbeat happens
    /// It's set in the callback that was set up a while ago, called teleopHeartbeatCb
    receivedHeartbeat.store(false);
    ros::spinOnce();
  }

  /// If the main loop's exited, signal to the controlling thread to stop running
  running.store(false);
  /// And then wait for it to finish before exiting
  controlThread.join();

  return 0;
}

#if USE_GAZEBO
/// This is code for spawning a model in gazebo, as the name implies
bool SpawnModel(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate) {
  // copy the model into a string to pass into the model spawner
  std::string model_path = "";
  std::stringstream model;

  /// Get the path from rosparams stuff
  if (!nhPrivate.getParam("model_path", model_path)) {
    ROS_ERROR("no model path specified");
    ROS_ERROR("path: %s", model_path.c_str());
    return -3;
  }
  ROS_INFO("opening model %s", model_path.c_str());
  {
    /// Open the file using an ifstream object
    auto model_file = std::ifstream(model_path.c_str());
    /// and then copy the model into the stringstream
    /// Yeah C++'s syntax is kind of weird here
    model << model_file.rdbuf();
  }

  // spawn model
  /// Not entirely sure why model_name is here, it might be useful later
  /// dunno
  std::string model_name = "";
  {
    /// Set up a service client to talk to the gazebo model spawning service
    auto model_spawner = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    /// Create the request/response object thing
    gazebo_msgs::SpawnModel sm;
    /// And copy the right things into it
    sm.request.model_name = "tesbot";
    sm.request.model_xml = model.str();
    sm.request.robot_namespace = "tesbot";
    // sm.request.initial_pose;

    /// Then call the spawner to make the model and check the response to see
    /// if it worked
    if (model_spawner.call(sm)) {
      if (sm.response.success) {
        ROS_INFO("robot spawn successful");
        model_name = sm.request.model_name;
      } else {
        ROS_ERROR("spawn attempt failed");
        ROS_ERROR("error message: %s", sm.response.status_message.c_str());
        return false;
      }
    } else {
      ROS_ERROR("unable to connect to model spawner");
      return false;
    }
  }
  return true;
}
#endif
