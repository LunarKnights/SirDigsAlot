#ifndef LK_CONTROLLER_H
#define LK_CONTROLLER_H

#include "ros/ros.h"

class LKController
{
public:
  LKController(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);
  ~LKController();

  void doStuff();

private:


protected:
  ros::NodeHandle &nh, &nhPrivate;
};

#endif
