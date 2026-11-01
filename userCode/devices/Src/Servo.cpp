//
// Created by LEGION on 2021/10/5.
//

#include "Servo.h"
namespace {
constexpr float kServoSpeedLimit = 100.0f;
constexpr float kServoPulseNeutralMs = 1.5f;
constexpr float kServoPulsePerSpeedMs = 0.005f;
constexpr float kServoPeriodMs = 20.0f;
}

const SERVO_HANDLE_T servoInfo[7]={
        {&htim1, TIM_CHANNEL_1},
        {&htim1, TIM_CHANNEL_2},
        {&htim1, TIM_CHANNEL_3},
        {&htim1, TIM_CHANNEL_4},
        {&htim8, TIM_CHANNEL_1},
        {&htim8, TIM_CHANNEL_2},
        {&htim8, TIM_CHANNEL_3},
};

uint32_t Servo::servo_IDs;
/**
 * @brief 舵机类的初始化函数，开启了相关定时器
 */
void Servo::Init() {
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim8);
}
/**
 * @brief 舵机的构造函数，进行舵机的初始化设置
 * @param servoInit 舵机的初始化结构体指针
 */
Servo::Servo(SERVO_INIT_T *servoInit) {
    deviceID = servoInit->servoID;
    deviceType = SERVO;

    angleLimit_Min = servoInit->angleLimit_Min;
    angleLimit_Max = servoInit->angleLimit_Max;

    servo_IDs |= deviceID;
    servoType = servoInit->servoType;
    duty = 0.0f;
    if(servoType == POSITION_180) {
        duty = AFFINE(0,180,0.025,0.125,servoInit->firstAngle);
    }else if(servoType == SPEED_360){
        duty = kServoPulseNeutralMs / kServoPeriodMs;
    }
    ApplyDuty(duty);
}

void Servo::stop() {
    if (stopFlag == 0){
        stopFlag = 1;
        auto pos = GET_SERVO_POS(deviceID);
        HAL_TIM_PWM_Stop(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
}

void Servo::ApplyDuty(float _duty) {
    auto pos = GET_SERVO_POS(deviceID);
    const auto &cfg = servoInfo[pos];
    const uint32_t periodTicks = cfg.handleTypeDef->Instance->ARR + 1u;
    __HAL_TIM_SET_COMPARE(cfg.handleTypeDef, cfg.timChannel,
                          static_cast<uint32_t>(periodTicks * _duty));
}
/**
 * @brief 设置舵机角度
 * @param _targetAngle
 */
void Servo::SetTargetAngle(float _targetAngle) {
    if (servoType != POSITION_180) {
        return;
    }
    INRANGE(_targetAngle,angleLimit_Min,angleLimit_Max);
    if(stopFlag == 1){
        stopFlag = 0;
        auto pos = GET_SERVO_POS(deviceID);
        HAL_TIM_PWM_Start(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
    targetAngle = _targetAngle;
}
/**
 * @设置舵机速度
 * @param _targetSpeed 舵机速度 (-100 到 100)
 */
void Servo::SetTargetSpeed(float _targetSpeed) {
    if (servoType != SPEED_360) {
        return;
    }
    INRANGE(_targetSpeed,-kServoSpeedLimit,kServoSpeedLimit); // (-100 到 100)
    if(stopFlag == 1) {
        stopFlag = 0;
        auto pos = GET_SERVO_POS(deviceID);
        HAL_TIM_PWM_Start(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
    targetSpeed = _targetSpeed;
}
/**
 * @brief 舵机的控制驱动函数
 */
void Servo::Handle() {
    if (servoType == POSITION_180){
        duty = AFFINE(0,180,0.025,0.125,targetAngle);
    }else if(servoType == SPEED_360){
        // 舵机的控制一般需要一个20ms左右的时基脉冲，该脉冲的高电平部分一般为0.5ms-2.5ms范围内的角度控制脉冲部分
        // 高电平为1毫秒~1.5毫秒时，舵机正转(1毫秒时正转速度最快，越接近1.5毫秒越慢，1.5毫秒时舵机停转)
        // 高电平为1.5毫秒时舵机停转，越接近2毫秒反转的速度越快，2毫秒时以最
        const float pulse_ms = kServoPulseNeutralMs + kServoPulsePerSpeedMs * targetSpeed;
        duty = pulse_ms / kServoPeriodMs;
    }else{
        return;
    }
    ApplyDuty(duty);
}

void Servo::ErrorHandle() {

}

