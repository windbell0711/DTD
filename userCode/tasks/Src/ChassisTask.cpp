//
// Created by LEGION on 2021/10/4.
//
#include "ChassisTask.h"
#include "ChassisKinematics.hpp"

#define SPEED_SCALE_ARG (0.5f)

PID_Regulator_t pidRegulator = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 120,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 16384,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 16384 //3508电机输出电流上限，可以调小，勿调大
};

MOTOR_INIT_t chassisMotorInit = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 19.0f,
        .ctrlType = SPEED_Single,
};

Motor CMFL(MOTOR_ID_1,&chassisMotorInit);//定义左前轮电机
Motor CMFR(MOTOR_ID_4,&chassisMotorInit);//定义右前轮电机
Motor CMBL(MOTOR_ID_2,&chassisMotorInit);//定义左后轮电机
Motor CMBR(MOTOR_ID_3,&chassisMotorInit);//定义右后轮电机

uint8_t ChassisStopFlag = 0;
float FBVelocity,LRVelocity,RTVelocity;

void ChassisStart(){

}
/**
 * @brief 底盘任务的处理函数，定时执行
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) in Device.cpp
 */
void ChassisHandle() {
    if(ChassisStopFlag == 0) {
        WheelsSpeedCalc(FBVelocity, LRVelocity, RTVelocity);
    }
    CMFL.Handle();
    CMFR.Handle();
    CMBL.Handle();
    CMBR.Handle();
}
/**
 * @brief 用于控制任务控制底盘速度
 * @param _fbV 底盘前后方向速度
 * @param _lrV 底盘左右方向速度
 * @param _rtV 底盘旋转速度
 */
void ChassisSetVelocity(float _fbV,float _lrV,float _rtV){
    ChassisStopFlag = 0;
    FBVelocity = SPEED_SCALE_ARG * _fbV;
    LRVelocity = SPEED_SCALE_ARG * _lrV;
    RTVelocity = SPEED_SCALE_ARG * _rtV;
}
/**
 * @brief 执行急停模式的底盘任务处理
 */
void ChassisStop(){
    ChassisStopFlag = 1;
    CMFL.Stop();
    CMFR.Stop();
    CMBL.Stop();
    CMBR.Stop();
}

/**
 * @brief 把教学骨架里的宏接到解算类上，保证参数只有一处来源
 */
static ChassisKinematics::Config MakeKinematicsConfig() {
    ChassisKinematics::Config cfg;
    cfg.wheelDiameter = WHEEL_DIAMETER;
    cfg.trackWidth    = TRACK_WIDTH;
    cfg.wheelBase     = WHEEL_BASE;
    cfg.dirFl         = WHEEL_DIR_FL;
    cfg.dirFr         = WHEEL_DIR_FR;
    cfg.dirBl         = WHEEL_DIR_BL;
    cfg.dirBr         = WHEEL_DIR_BR;
    cfg.maxLinear     = CHASSIS_MAX_LINEAR;
    cfg.maxRotRpm     = CHASSIS_MAX_ROT_RPM;
    return cfg;
}

static const ChassisKinematics kKinematics{MakeKinematicsConfig()};

/**
 * @brief 底盘速度 -> 四个轮子目标转速（实现见 ChassisKinematics.cpp）
 * @param fbVelocity 前后速度，向前为正，m/s
 * @param lrVelocity 左右速度，向右为正，m/s
 * @param rtVelocity 旋转速度，顺时针（车头向右转）为正，rpm
 */
void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity) {
    const WheelTargetRpm wheel = kKinematics.solve(fbVelocity, -lrVelocity, rtVelocity);

    CMFL.SetTargetSpeed(wheel.fl);
    CMFR.SetTargetSpeed(-wheel.fr);
    CMBL.SetTargetSpeed(wheel.bl);
    CMBR.SetTargetSpeed(-wheel.br);
}

#ifdef WHEEL_SELFTEST
/**
 * @brief 单轮点动自检：只驱动 SELFTEST_WHEEL 号轮子，其余三轮断电
 * @note  直接调用 Handle() 刷新电流，不经过遥控器，也不经过逆运动学
 */
void ChassisSelfTest() {
    Motor *wheels[4] = {&CMFL, &CMFR, &CMBL, &CMBR};

    for (uint8_t i = 0; i < 4; ++i) {
        if (i == (SELFTEST_WHEEL - 1)) {
            wheels[i]->SetTargetSpeed(SELFTEST_RPM);
        } else {
            wheels[i]->Stop();
        }
        wheels[i]->Handle();
    }
}
#endif
