//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"

#include "RemoteControl.h"

PID_Regulator_t userPidRegulator = {
        .kp = 60,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 10000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 10000 //2006电机输出电流上限，可以调小，勿调大
};

MOTOR_INIT_t userMotorInit = {
        .speedPIDp = &userPidRegulator,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 36.0f,
        .ctrlType = SPEED_Single,
};

// 在这里初始化除了4个M3508轮电机之外的电机
Motor UserMotor(MOTOR_ID_5,&userMotorInit);

/***
 * 在这里放入xxx.stop()即可使舵机,电机在遥控器急停挡位断电
 */
void UserStop(){
    UserMotor.Stop();
    TurnLServo.stop();
}

/***
 * 在这里写入初始化内容
 */
void UserInit(){
    TurnLServo.SetTargetSpeed(0.0f);
}

/***
 * 用户自定义任务主循环
 */
void UserHandle(){
    UserMotor.Handle();
    TurnLServo.Handle();
    switch (RemoteControl::rcInfo.sLeft) {
    case UP_POS:
        TurnLServo.SetTargetSpeed(50.0f);
        break;
    case MID_POS:
        TurnLServo.SetTargetSpeed(25.0f);
        break;
    case DOWN_POS:
        TurnLServo.SetTargetSpeed(0.0f);
        break;
    }
}