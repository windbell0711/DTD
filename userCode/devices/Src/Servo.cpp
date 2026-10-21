//
// Created by LEGION on 2021/10/5.
//

#include "Servo.h"


SERVO_INIT_T aServoInit = {
    .servoType = SPEED_360,
    .servoID = SERVO_ID_1,
    .firstAngle = 0,
    .angleLimit_Min = 0,
    .angleLimit_Max = 180
};
Servo TurnLServo = Servo(&aServoInit);


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
    float duty;
    if(servoType == POSITION_180) {
        duty = AFFINE(0,180,0.025,0.125,servoInit->firstAngle);
    }else if(servoType == SPEED_360){
        duty = 0.075;
    }
    auto pos = GET_SERVO_POS(deviceID);

    __HAL_TIM_SET_COMPARE(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel,
            htim1.Instance->ARR*duty);

}

void Servo::stop() {
    if (stopFlag == 0){
        stopFlag = 1;
        auto pos = GET_SERVO_POS(deviceID);
        HAL_TIM_PWM_Stop(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
}
/**
 * @brief 设置舵机角度
 * @param _targetAngle
 */
void Servo::SetTargetAngle(float _targetAngle) {
    //TODO 检查舵机类型,检查角度限位
    if (servoType != POSITION_180) {
        return;
    }
    if (targetAngle < angleLimit_Min || targetAngle > angleLimit_Max) {
        return;
    }
    if(stopFlag == 1){
        stopFlag = 0;
        auto pos = GET_SERVO_POS(deviceID);
        HAL_TIM_PWM_Start(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
    INRANGE(_targetAngle,angleLimit_Min,angleLimit_Max);
    targetAngle = _targetAngle;
}
/**
 * @设置舵机速度
 * @param _targetSpeed 舵机速度
 */
void Servo::SetTargetSpeed(float _targetSpeed) {
    //TODO 检查舵机类型
    if (servoType != SPEED_360) {
        return;
    }
    if(stopFlag == 1){
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
        auto pos = GET_SERVO_POS(deviceID);

        __HAL_TIM_SET_COMPARE(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel,
                              htim1.Instance->ARR*duty);

    }else if(servoType == SPEED_360){
        float s = targetSpeed;              // -100..100
        float pulse_ms;

        if (s <= -100.0f) {
            pulse_ms = 0.5f;                // 饱和：最快顺时针
        } else if (s < 0.0f) {
            // 顺时针线性区：s=-100→1.0ms, s=0→1.5ms
            // 注意：表格说 ≤1.0ms 都已饱和，这里用 1.0ms 作线性区上界
            pulse_ms = 1.5f + s * 0.005f;   // -100→1.0, 0→1.5
        } else if (s == 0.0f) {
            pulse_ms = 1.5f;                // 停止
        } else if (s < 100.0f) {
            // 逆时针线性区（若舵机不支持逆时针，这一支删掉/钳到1.5ms）
            pulse_ms = 1.5f + s * 0.005f;   // 0→1.5, 100→2.0
        } else {
            pulse_ms = 2.5f;                // 饱和：最快逆时针
        }

        duty = pulse_ms / 20.0f;            // 周期 20ms

        auto pos = GET_SERVO_POS(deviceID);

        __HAL_TIM_SET_COMPARE(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel,
                      htim1.Instance->ARR*duty);
    }
}

void Servo::ErrorHandle() {

}

