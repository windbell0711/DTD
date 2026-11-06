//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"
#include "ChassisTask.h"
#include "PushRod.h"
#include "BusServo.h"

#if defined(BUS_SERVO_TEST) || defined(BUS_SERVO_SET_ID)
static BusServo *g_busServo = nullptr;
#endif

// 总线舵机自检结果：bit0 = ID1 应答, bit1 = ID2 应答（Keil Watch 里直接看）
volatile uint8_t g_busServoStatus = 0;

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

SERVO_INIT_T ShovelL_Servo_Init = {
        .servoType = SPEED_360,
        .servoID = SERVO_ID_3,
        .firstAngle = 0,
        .angleLimit_Min = 0,
        .angleLimit_Max = 180
};
Servo ShovelLServo(&ShovelL_Servo_Init);

SERVO_INIT_T ShovelR_Servo_Init = {
        .servoType = SPEED_360,
        .servoID = SERVO_ID_4,
        .firstAngle = 0,
        .angleLimit_Min = 0,
        .angleLimit_Max = 180
};
Servo ShovelRServo(&ShovelR_Servo_Init);

/***
 * 在这里放入xxx.stop()即可使舵机,电机在遥控器急停挡位断电
 */
void UserStop(){
    UserMotor.Stop();

    pushrod_off();

    TurnLServo.stop();
    TurnRServo.stop();

    ShovelLServo.stop();
    ShovelRServo.stop();
}

/***
 * 在这里写入初始化内容
 */
void UserInit(){
    TurnLServo.SetTargetSpeed(0.0f);
    TurnRServo.SetTargetSpeed(0.0f);
    ShovelLServo.SetTargetSpeed(0.0f);
    ShovelRServo.SetTargetSpeed(0.0f);
    TurnLServo.Handle();
    TurnRServo.Handle();
    ShovelLServo.Handle();
    ShovelRServo.Handle();
#if defined(BUS_SERVO_TEST) || defined(BUS_SERVO_SET_ID)
    // 放在函数内构造：此时 UART 已经初始化完毕
    static BusServo servo(BUS_SERVO_UART);
    g_busServo = &servo;
#endif
#ifdef BUS_SERVO_SET_ID
    // 只接一个舵机时使用：把 ID 1 改成 ID 2
    if (servo.ping(1)) {
        servo.setID(1, 2);
    }
#endif
#ifdef BUS_SERVO_TEST
    uint8_t st = 0;
    if (servo.ping(1)) {
        st |= 0x01;
    }
    if (servo.ping(2)) {
        st |= 0x02;
    }
    g_busServoStatus = st;
#endif
}

/***
 * 用户自定义任务主循环
 */
void UserHandle(){
    UserMotor.Handle();
    TurnLServo.Handle();
    TurnRServo.Handle();
    ShovelLServo.Handle();
    ShovelRServo.Handle();

#ifdef BUS_SERVO_TEST
    // 每 20 ms 同步下发两个舵机的目标位置：1 号正向扫，2 号反向扫（镜像）
    static uint32_t lastTick = 0;
    static int16_t target = 1000;
    static int8_t dir = 1;

    uint32_t now = HAL_GetTick();
    if (g_busServo && ((now - lastTick) >= 20u)) {
        lastTick = now;

        const uint8_t ids[2] = {1, 2};
        const int16_t pos[2] = {target, (int16_t)(4095 - target)};
        g_busServo->syncWritePos(ids, pos, 600, 30, 2);

        target = (int16_t)(target + dir * 40);
        if (target >= 3000) { target = 3000; dir = -1; }
        if (target <= 1000) { target = 1000; dir = 1; }
    }
#endif
}
