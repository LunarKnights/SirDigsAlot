#include <ros/ros.h>

#include <atomic>
#include <thread>

#include "lk_base/lk_hardware.h"
#include "controller_manager/controller_manager.h"


// TODO: add estop

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lk_base");
  ros::NodeHandle nh, nhPriv("~");

  lk_base::LKHardware lk_hw(nh, nhPriv);
  controller_manager::ControllerManager cm(&lk_hw, nh);
  std::atomic<bool> running(true);

  std::thread controlThread([&]() {
      auto r = ros::Rate(50);
      while (running)
      {
        auto curTime = ros::Time::now();
        r.sleep();
        lk_hw.updateJointsFromHardware(r.cycleTime().toSec());
        cm.update(curTime, r.cycleTime());
        lk_hw.writeCommandsToHardware();
      }
  });

  ros::spin();
  running.store(false);

  controlThread.join();

  return 0;
}
