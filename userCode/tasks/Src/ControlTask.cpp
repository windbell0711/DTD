//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"
#include "PushRod.h"

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

    if (RemoteControl::rcInfo.SD != UP_POS) {
        // SD 下档急停模式，改过了
        ChassisStop();
        UserStop();
    } else {
        // 底盘运动始终开启
        ChassisSetVelocity(ApplyDeadzone(RemoteControl::rcInfo.right_col) * CHASSIS_MAX_LINEAR,
                           ApplyDeadzone(RemoteControl::rcInfo.right_rol) * CHASSIS_MAX_LINEAR,
                           ApplyDeadzone(RemoteControl::rcInfo.left_rol) * CHASSIS_MAX_ROT_RPM);
        if (RemoteControl::rcInfo.SA == UP_POS) {
            // 前同步带控制
            switch (RemoteControl::rcInfo.SB) {
                // case UP_POS:   pushrod_out();  break;
                // case MID_POS:  pushrod_in();   break;
                // case DOWN_POS: pushrod_off();  break;
            }
            // 后同步带控制
            switch (RemoteControl::rcInfo.SC) {
                // case UP_POS:   pushrod_out();  break;
                // case MID_POS:  pushrod_in();   break;
                // case DOWN_POS: pushrod_off();  break;
            }
        } else if (RemoteControl::rcInfo.SA == DOWN_POS) {
            // 推杆控制
            switch (RemoteControl::rcInfo.SB) {
                case UP_POS:   pushrod_out();  break;
                case MID_POS:  pushrod_in();   break;
                case DOWN_POS: pushrod_off();  break;
            }
            // 总线舵机控制
            switch (RemoteControl::rcInfo.SC) {
                // case UP_POS:   pushrod_out();  break;
                // case MID_POS:  pushrod_in();   break;
                // case DOWN_POS: pushrod_off();  break;
            }
        } else {
            // 未知危险状态
            ChassisStop();
            UserStop();
        }
        // switch (RemoteControl::rcInfo.SB) {
        // case UP_POS: //左侧一档
        //     ChassisSetVelocity(ApplyDeadzone(RemoteControl::rcInfo.right_col) * CHASSIS_MAX_LINEAR,
        //                        ApplyDeadzone(RemoteControl::rcInfo.right_rol) * CHASSIS_MAX_LINEAR,
        //                        ApplyDeadzone(RemoteControl::rcInfo.left_rol) * CHASSIS_MAX_ROT_RPM);
        //     break;
        // case MID_POS: //左侧二档
        //
        //     break;
        // case DOWN_POS: //左侧三档
        //
        //     break;
        // default:
        //     break;
        // }
    }
}

