#ifndef COMMON_H_
#define COMMON_H_

// TODO: get these from the URDF somehow
constexpr float kWheelSeparation = 3.1415f;
constexpr float kWheelRadius = 0.1016f;

constexpr int kNumJoints = 4;

constexpr char kRobotName[] = "tesbot";

constexpr char kJointLeftBackName[] = "base_to_left_back_wheel";
constexpr char kJointLeftFrontName[] = "base_to_left_front_wheel";
constexpr char kJointRightBackName[] = "base_to_right_back_wheel";
constexpr char kJointRightFrontName[] = "base_to_right_front_wheel";

constexpr char kBaseLinkName[] = "tesbot::base_link";

constexpr char kLinkLeftBackName[] = "tesbot::left_back_wheel";
constexpr char kLinkLeftFrontName[] = "tesbot::left_front_wheel";
constexpr char kLinkRightBackName[] = "tesbot::right_back_wheel";
constexpr char kLinkRightFrontName[] = "tesbot::right_front_wheel";

constexpr const char* kJointNames[kNumJoints] = {
    kJointLeftFrontName,
    kJointLeftBackName,
    kJointRightFrontName,
    kJointRightBackName
};
constexpr const char* kLinkNames[kNumJoints] = {
    kLinkLeftFrontName,
    kLinkLeftBackName,
    kLinkRightFrontName,
    kLinkRightBackName
};

class BaseController;
class PIDController;
#endif
