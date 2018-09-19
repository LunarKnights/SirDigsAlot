#include "ctre/Phoenix.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include "Platform-linux-socket-can.h"
#include <math.h>

// SDL code from https://gist.github.com/fabiocolacio/423169234b8daf876d8eb75d8a5f2e95

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
int kTimeoutMs = 30;
int programCounter = 0;
int main() {

    ctre::phoenix::platform::can::SetCANInterface("can0");
    //ctre::phoenix::
    TalonSRX * talon = new TalonSRX(1);

    talon->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,kTimeoutMs);
    talon->SetSensorPhase(true);

    talon->ConfigNominalOutputForward(0,kTimeoutMs);
    talon->ConfigNominalOutputReverse(0,kTimeoutMs);

    talon->ConfigPeakOutputForward(1,kTimeoutMs);
    talon->ConfigPeakOutputReverse(-1,kTimeoutMs);

    talon->Config_kF(0,0.1097,kTimeoutMs);
    talon->Config_kP(0,.22,kTimeoutMs);
    talon->Config_kI(0,0,kTimeoutMs);
    talon->Config_kD(0,0,kTimeoutMs);


    talon->SetStatusFramePeriod(StatusFrame::Status_1_General_,5,kTimeoutMs);
    double theta = 0;
while(1){
     if(programCounter++%10==0){
         //std::cout << "T: " << talon->GetTemperature() << std::endl;
         std::cout << "T: " << talon->GetClosedLoopTarget()<<" A: "<< talon->GetSelectedSensorVelocity()<<" E: "<<talon->GetClosedLoopError() <<std::endl;
     }
    ctre::phoenix::platform::FeedWatchDog(100);
    //talon->Set(ControlMode::PercentOutput, 0.1f);
    theta++;
    double targetVelocity_UnitsPer100ms = sin(theta/20) * 132 * 4096 / 600;
    talon->Set(ControlMode::Velocity,targetVelocity_UnitsPer100ms);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

}
    return 0;
}



