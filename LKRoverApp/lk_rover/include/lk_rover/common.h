#ifndef LK_COMMON_H
#define LK_COMMON_H

constexpr int kNumWheels = 4;
constexpr const char* kWheelNames[] = {
  "base_to_left_front_wheel",
  "base_to_left_back_wheel",
  "base_to_right_front_wheel",
  "base_to_right_back_wheel"
};

constexpr double kMaxTorque = 20.0;
// 90 pulses per revolution according to the Vex datasheet: 
// https://content.vexrobotics.com/docs/instructions/276-2156-instr-0312.pdf
constexpr double kEncoderToRadians = 2.0*3.141592/90.0;

#endif
