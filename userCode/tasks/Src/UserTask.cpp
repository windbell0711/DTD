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

SERVO_INIT_T TurnL_Servo_Init = {
        .servoType = SPEED_360,
        .servoID = SERVO_ID_1,
        .firstAngle = 0,
        .angleLimit_Min = 0,
        .angleLimit_Max = 180
};
Servo TurnLServo(&TurnL_Servo_Init);

SERVO_INIT_T TurnR_Servo_Init = {
        .servoType = SPEED_360,
        .servoID = SERVO_ID_2,
        .firstAngle = 0,
        .angleLimit_Min = 0,
        .angleLimit_Max = 180
};
Servo TurnRServo(&TurnR_Servo_Init);

/***
 * 在这里放入xxx.stop()即可使舵机,电机在遥控器急停挡位断电
 */
void UserStop(){
    UserMotor.Stop();
    TurnLServo.stop();
    TurnRServo.stop();
}

/***
 * 在这里写入初始化内容
 */
void UserInit(){
    TurnLServo.SetTargetSpeed(0.0f);
    TurnRServo.SetTargetSpeed(0.0f);
    TurnLServo.Handle();
    TurnRServo.Handle();
}

/***
 * 用户自定义任务主循环
 */
void UserHandle(){
    if (!RemoteControl::isOnline() || RemoteControl::rcInfo.sRight == DOWN_POS) {
        UserStop();
    } else {
        switch (RemoteControl::rcInfo.sLeft) {
        case UP_POS:
            TurnLServo.SetTargetSpeed(90.0f);
            TurnRServo.SetTargetSpeed(-90.0f);
            break;
        case MID_POS:
            TurnLServo.SetTargetSpeed(85.0f);
            TurnRServo.SetTargetSpeed(-85.0f);
            break;
        case DOWN_POS:
            TurnLServo.SetTargetSpeed(0.0f);
            TurnRServo.SetTargetSpeed(0.0f);
            break;
        default:
            TurnLServo.SetTargetSpeed(0.0f);
            TurnRServo.SetTargetSpeed(0.0f);
            break;
        }
    }

    UserMotor.Handle();
    TurnLServo.Handle();
    TurnRServo.Handle();
}
