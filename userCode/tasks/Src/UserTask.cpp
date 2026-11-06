//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"
#include "RemoteControl.h"
#include "ChassisTask.h"

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
                       (int)RemoteControl::rcInfo.SB,
                       (int)RemoteControl::rcInfo.SC,
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

}

/***
 * 用户自定义任务主循环
 */
void UserHandle(){
    UserMotor.Handle();
#ifdef RC_DEBUG
    UserDebugPrint();
#endif
}