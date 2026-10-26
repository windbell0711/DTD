#include "ChassisKinematics.hpp"

namespace {
constexpr float kPi       = 3.14159265358979f;
constexpr float kRpmToRad = kPi / 30.0f;   // 1 rpm = π/30 rad/s
constexpr float kRadToRpm = 30.0f / kPi;   // 1 rad/s = 30/π rpm
}

ChassisKinematics::ChassisKinematics() : cfg_() {
}

ChassisKinematics::ChassisKinematics(const Config &cfg) : cfg_(cfg) {
}

WheelTargetRpm ChassisKinematics::solve(float vx, float vy, float wzRpm) const {
    const float radius = cfg_.wheelDiameter * 0.5f;
    const float k      = (cfg_.trackWidth + cfg_.wheelBase) * 0.5f;  // 自转等效半径
    const float wz     = wzRpm * kRpmToRad;                          // rpm -> rad/s

    // 四个轮子的接地点线速度，m/s
    const float fl = vx - vy + k * wz;
    const float fr = vx + vy - k * wz;
    const float bl = vx + vy + k * wz;
    const float br = vx - vy - k * wz;

    WheelTargetRpm out;
    out.fl = cfg_.dirFl * (fl / radius) * kRadToRpm;
    out.fr = cfg_.dirFr * (fr / radius) * kRadToRpm;
    out.bl = cfg_.dirBl * (bl / radius) * kRadToRpm;
    out.br = cfg_.dirBr * (br / radius) * kRadToRpm;
    return out;
}

WheelTargetRpm ChassisKinematics::fromStick(float fwd, float strafe, float turn) const {
    // 中位附近的噪声直接归零，避免松手后车自己爬
    const auto applyDeadzone = [this](float v) {
        return (v > -cfg_.deadzone && v < cfg_.deadzone) ? 0.0f : v;
    };

    return solve(applyDeadzone(fwd) * cfg_.maxLinear,
                 applyDeadzone(strafe) * cfg_.maxLinear,
                 applyDeadzone(turn) * cfg_.maxRotRpm);
}
