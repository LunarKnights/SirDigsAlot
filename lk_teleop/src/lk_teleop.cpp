#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <linux/joystick.h>

#include <algorithm>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
// lots of code copied from http://www.cs.uleth.ca/~holzmann/C/system/ttyraw.c
struct termios origTermios;

int main(int argc, char** argv)
{
  // start the node lk_teleop
  ros::init(argc, argv, "lk_teleop");
  // create the node handle
  auto nh = ros::NodeHandle();

  auto nhPriv = ros::NodeHandle("~");
  // init publisher for cmd_vel. This is for telling the robot where to go
  ros::Publisher pubBase = nh.advertise<geometry_msgs::Twist>("/lk_velocity_controller/cmd_vel", 1);
  // init publisher for lk_spin_controller/command, tell the ladder how fast to spin/direction
  ros::Publisher pubLadderSpin = nh.advertise<std_msgs::Float64>("/lk_spin_controller/command", 1);
  // init publisher ofr lk_dump_controller/command, tell the bucket where to go(up/down)
  ros::Publisher pubBucketLift = nh.advertise<std_msgs::Float64>("/lk_dump_controller/command", 1);
  // init publisher for lk_ladder_controller/command, tell the bucket ladder where to go (up/down)
  ros::Publisher pubLadderLift = nh.advertise<std_msgs::Float64>("/lk_ladder_controller/command", 1);
  // init publisher for /lk_flap_controller/command, tell the flap where to go (up/down)
  ros::Publisher pubFlapLift = nh.advertise<std_msgs::Float64>("/lk_flap_controller/command", 1);
  // init publisher for heartbeat, used to keep track of connectivity to ros
  ros::Publisher heartbeat = nh.advertise<std_msgs::Bool>("/heartbeat", 1);
  // initialize the joystick, if it is not reachable, exit and display message
  int joy_fd = open("/dev/input/js0", O_RDONLY);
  if (joy_fd == -1)
  {
    fprintf(stderr, "unable to open controller\n");
    exit(-1);
  }
  // init check to make sure that the controller found is the right one. print out io info
  int numAxes = 0, numButtons = 0;
  char joystickName[80];
  ioctl(joy_fd, JSIOCGAXES, &numAxes);
  ioctl(joy_fd, JSIOCGBUTTONS, &numButtons);
  ioctl(joy_fd, JSIOCGNAME(80), &joystickName);
  // init array for axes and buttons on the controller to save values
  int* axes = new int[numAxes];
  char* buttons = new char[numButtons];
  // tell console that the joystick was found with the information displayed
  ROS_INFO("Found joystick %s with %d axes and %d buttons", joystickName, numAxes, numButtons);
  // init strcuct for joystick
  struct js_event js;
  float bucketPos = 0.0;
  float ladderPos = 0.0;
  float flapPos = 0.0;
  // wait for start button to start teleop
  fd_set set;
  struct timeval timeout;
  bool started = false;
  ROS_INFO("Waiting for start...");
  // wait for joystick input, if start button, start, if stop button stop
  while (!started && ros::ok())
  {
    FD_ZERO(&set);
    FD_SET(joy_fd, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000;
    int rv = select(joy_fd + 1, &set, NULL, NULL, &timeout);

    if (rv > 0)
    {
      if (read(joy_fd, &js, sizeof(js)) < 0)
      {
        fprintf(stderr, "unable to read controller\n");
        exit(-2);
      }
      switch (js.type & ~JS_EVENT_INIT)
      {
        case JS_EVENT_AXIS:
          axes[js.number] = js.value;
          break;
        case JS_EVENT_BUTTON:
          buttons[js.number] = js.value;
          if (buttons[7])
          {
            started = true;
          }
          if (buttons[6])
          {
            ros::shutdown();
          }

          break;
      }
    }
    ros::spinOnce();
  }
  // display start detected when loop is broken
  ROS_INFO("Start detected! Beginning transmission..");
  // init heartbeat for publisher
  std_msgs::Bool heartbeatMsg;
  heartbeatMsg.data = true;
  // while ros is running
  while (ros::ok())
  {
    // printf("loop\n");
    heartbeat.publish(heartbeatMsg);

    FD_ZERO(&set);
    FD_SET(joy_fd, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000;
    int rv = select(joy_fd + 1, &set, NULL, NULL, &timeout);
    if (rv > 0)
    {
      // make sure joystick is still connected
      if (read(joy_fd, &js, sizeof(js)) < 0)
      {
        fprintf(stderr, "unable to read controller\n");
        exit(-2);
      }
      // handle joystick events
      switch (js.type & ~JS_EVENT_INIT)
      {
        case JS_EVENT_AXIS:
          axes[js.number] = js.value;
          // if the joystick has been moved in the x or y direction do this
          if (js.number == 0 || js.number == 1)
          {
            float forceLin = axes[1];
            float forceAng = axes[0];
            auto sgn = [](float x) -> float {
              if (x > 0.0)
                return 1.0;
              else if (x < 0.0)
                return -1.0;
              else
                return 0.0;
            };
            // create threshold for cancelling out noise for slight changes in joystick direction, make it more digital
            // rather than analog for better control with the diff drive controller on our very long base
            if (forceLin <= 0.2 * std::abs(forceAng) && forceLin >= -0.2 * std::abs(forceAng))
              forceLin = 0;
            else
              forceLin -= 0.2 * std::abs(forceAng) * sgn(forceLin);
            if (forceAng <= 0.2 * std::abs(forceLin) && forceAng >= -0.2 * std::abs(forceLin))
              forceAng = 0;
            else
              forceAng -= 0.2 * std::abs(forceLin) * sgn(forceAng);
            geometry_msgs::Twist msg = {};
            // lower the output of the publisher values -20 for linear and -5 for angular gives ideal results (32767 is
            // xbox 1 constant)
            msg.linear.x = forceLin / (-20 * 32767.0);
            msg.angular.z = forceAng / (-5 * 32767.0);
            // publish message
            pubBase.publish(msg);
          }
          // if the left bumper has been pressed start digging
          if (js.number == 5)
          {
            int force = axes[5];
            std_msgs::Float64 msg;
            // add threshold for bumper so that it doesn't read until pressed a cetain amount, .75 is ideal (32767 is
            // xbox 1 constant)
            if (force >= 0)
              msg.data = (force / (32767.0) * 0.75);
            // publish message
            pubLadderSpin.publish(msg);
          }
          // if left bumper is pressed turn the bucket ladder the other way
          if (js.number == 2)
          {
            int force = axes[2];
            std_msgs::Float64 msg;
            // add threshold for bumper so that it doesn't read until pressed a certain amount, .75 is ideal (32767 is
            // xbox 1 constant)
            if (force >= 0)
              msg.data = (-force / (32767.0) * 0.75);
            pubLadderSpin.publish(msg);
          }

          break;
        // check for flap lift button, not actually used
        case JS_EVENT_BUTTON:
          buttons[js.number] = js.value;
          if (js.number == 8)
          {
            float pos = buttons[8];
            std_msgs::Float64 msg;
            msg.data = pos;
            pubFlapLift.publish(msg);
          }
          // check for shutdown button, 6 aligns with the select button
          if (buttons[6])
          {
            ros::shutdown();
          }

          break;
      }
// print out axes and button values
#if 0
	    for(int i = 0; i < numAxes; ++i)
	      printf("a %d:%6d ", i, axes[i]);
	    for(int i = 0; i < numButtons; ++i)
	      printf("b %d: %d ", i, buttons[i]);
	    printf("  \r");
	    fflush(stdout);
#endif
    }
    else if (rv == 0)
    {
      // printf("timeout\n");
      // if buttons "y", "b", "a", or "x" are pressed do this
      if (buttons[0] || buttons[1] || buttons[2] || buttons[3])
      {
        // if button "y" is pressed add to bucket position, if "b" is pressed subtract
        if (buttons[1] || buttons[3])
        {
          // figure bucket position and multiply by gain amount to raise the position accuracy, .001 is ideal
          // the range of ladderPos and bucketPos are both measured in meters, so the 20cm extension is equal to 0.20
          // this limits the user's input into roughly the right range

          // NOTE: this doesn't affect the actual range; we cannot go outside the physical limitations of the robot
          // or the range set in the ROS controller because
          // the ROS controller appears to clamp commands to within the range it gets from the URDF of the robot
          bucketPos += 0.001 * (buttons[3] - buttons[1]);
          if (bucketPos > 0.25)
            bucketPos = 0.25;
          if (bucketPos < -0.05)
            bucketPos = -0.05;
          std_msgs::Float64 msg;
          msg.data = bucketPos;
          pubBucketLift.publish(msg);
        }
        // if button "x" is pressed add to the ladder position, if "a" is pressed subtract
        if (buttons[2] || buttons[0])
        {
          // figure ladder position and multiply by gain amount to raise the position accuracy, .01 is ideal
          ladderPos += 0.01 * (buttons[2] - buttons[0]);
          // the range of ladderPos and bucketPos are both measured in meters, so the 20cm extension is equal to 0.20
          // this limits the user's input into roughly the right range

          // NOTE: this doesn't affect the actual range; we cannot go outside the physical limitations of the robot
          // or the range set in the ROS controller because
          // the ROS controller appears to clamp commands to within the range it gets from the URDF of the robot
          if (ladderPos > 0.25)
            ladderPos = 0.25;
          if (ladderPos < -0.05)
            ladderPos = -0.05;
          std_msgs::Float64 msg;
          msg.data = ladderPos;
          pubLadderLift.publish(msg);
        }
      }
    }
    else
    {
      fprintf(stderr, "select() error\n");
    }
    // this is ros standar clock cycle, must be spun once per loop to do what it is supposed to;
    ros::spinOnce();
  }
  // close stream
  close(joy_fd);
  return 0;
}
