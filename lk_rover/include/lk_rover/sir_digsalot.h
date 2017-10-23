#ifndef SIR_DIGSALOT_H
#define SIR_DIGSALOT_H

#include "lk_controller.h"

class SirDigsalot : LKController
{
public:
   SirDigsalot(ros::NodeHandle& nh, ros::NodeHandle& nhPrivate);
   ~SirDigsalot();

   void DoStuff();

private:
};

#endif
