//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"

/**
 * @brief 摇杆死区：中位附近的噪声直接归零，避免松手后车自己爬
 */
static float ApplyDeadzone(float value) {
    const float deadzone = 0.02f;
    if ((value > -deadzone) && (value < deadzone)) {
        return 0.0f;
    }
    return value;
}

void CtrlHandle(){
#ifdef WHEEL_SELFTEST
    //调试：忽略遥控器，单轮点动（见 ChassisTask.cpp 的 ChassisSelfTest）
    ChassisSelfTest();
    return;
#endif

    if (!RemoteControl::isOnline()){
        //遥控器未开机/掉线：立即停车，不能按最后一帧指令继续跑
        ChassisStop();
        UserStop();
        return;
    }

    if (RemoteControl::rcInfo.sRight == DOWN_POS){
        //右侧下三档，急停模式，请勿更改！！！
        ChassisStop();
        UserStop();
    } else {
        //其他正常模式
        switch (RemoteControl::rcInfo.sLeft)
        {
        case UP_POS: //左侧一档
            ChassisSetVelocity(ApplyDeadzone(RemoteControl::rcInfo.right_col) * CHASSIS_MAX_LINEAR,
                               ApplyDeadzone(RemoteControl::rcInfo.right_rol) * CHASSIS_MAX_LINEAR,
                               ApplyDeadzone(RemoteControl::rcInfo.left_rol) * CHASSIS_MAX_ROT_RPM);
            break;
        case MID_POS: //左侧二档
            //离开底盘档必须停车，否则会保持上一次的摇杆速度继续跑
            ChassisStop();
            break;
        case DOWN_POS: //左侧三档
            ChassisStop();
            break;
        default:
            break;
        }
    }
}

