//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"
#include "RemoteControl.h"
#include "ChassisTask.h"
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

#ifdef RC_DEBUG
/**
 * @brief 串口调试输出：档位原始值、归类结果、摇杆、四轮目标转速、四轮实际转速
 * @note  100 ms 一次；USB 没插时 CDC_Transmit_FS 会返回 BUSY，直接丢帧即可
 * @note  t= 是目标转速（运动学输出），rpm= 是实际反馈；没接电机时 rpm 恒为 0 是正常的
 */
static void UserDebugPrint() {
    static uint32_t lastTick = 0;
    static char buf[160];

    uint32_t now = HAL_GetTick();
    if ((now - lastTick) < 100u) {
        return;
    }
    lastTick = now;

    int len = snprintf(buf, sizeof(buf),
                       "sw=%u,%u cls=%d,%d on=%d rc=%d,%d,%d,%d t=%d,%d,%d,%d rpm=%d,%d,%d,%d\r\n",
                       (unsigned int)RemoteControl::rc_ctrl.rc.s[0],
                       (unsigned int)RemoteControl::rc_ctrl.rc.s[1],
                       (int)RemoteControl::rcInfo.sLeft,
                       (int)RemoteControl::rcInfo.sRight,
                       (int)RemoteControl::isOnline(),
                       (int)(RemoteControl::rcInfo.right_col * 1000.0f),
                       (int)(RemoteControl::rcInfo.right_rol * 1000.0f),
                       (int)(RemoteControl::rcInfo.left_rol * 1000.0f),
                       (int)(RemoteControl::rcInfo.left_col * 1000.0f),
                       (int)CMFL.targetSpeed,
                       (int)CMFR.targetSpeed,
                       (int)CMBL.targetSpeed,
                       (int)CMBR.targetSpeed,
                       (int)CMFL.state.speed,
                       (int)CMFR.state.speed,
                       (int)CMBL.state.speed,
                       (int)CMBR.state.speed);

    if (len > 0) {
        CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
    }
}
#endif

/***
 * 在这里放入xxx.stop()即可使舵机,电机在遥控器急停挡位断电
 */
void UserStop(){
    UserMotor.Stop();
}

/***
 * 在这里写入初始化内容
 */
void UserInit(){
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
#ifdef RC_DEBUG
    UserDebugPrint();
#endif
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