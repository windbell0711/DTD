//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"
#include "PushRod.h"

/**
 * @brief 摇杆死区：中位附近的噪声直接归零，避免松手后车自己爬
 */
static float ApplyDeadzone(float value, float deadzone) {
    if ((value > -deadzone) && (value < deadzone)) {
        return 0.0f;
    }
    return value;
}

/**
 * @brief 同步带一对舵机：上档往外、中档停、下档往里（镜像安装）
 */
static void BeltPair(SWITCH_STATE_E sw, Servo &left, Servo &right, float speed) {
    switch (sw) {
        case UP_POS:
            left.SetTargetSpeed(speed);
            right.SetTargetSpeed(-speed);
            break;
        case MID_POS:
            left.SetTargetSpeed(0.0f);
            right.SetTargetSpeed(0.0f);
            break;
        case DOWN_POS:
            left.SetTargetSpeed(-speed);
            right.SetTargetSpeed(speed);
            break;
        default:
            break;
    }
}

/**
 * @brief 推杆：左摇杆上下，阈值大，避免误触
 */
static void PushrodByStick(float upDown) {
    if (upDown > PLATE_PUSHROD_THRESHOLD) {
        pushrod_all_out();
    } else if (upDown < -PLATE_PUSHROD_THRESHOLD) {
        pushrod_all_in();
    } else {
        pushrod_all_off();
    }
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
        // SD 下档：失能
        ChassisStop();
        UserStop();
        return;
    }

    /* 同步带：SB 一个拨杆同时控制四颗舵机（上=外、中=停、下=里） */
    BeltPair(RemoteControl::rcInfo.SB, TurnLServo, TurnRServo,
             (float)SERVO_REAR_SPEED_PERC);
    BeltPair(RemoteControl::rcInfo.SB, ShovelLServo, ShovelRServo,
             (float)SERVO_FRONT_SPEED_PERC);

    if (RemoteControl::rcInfo.SA == DOWN_POS) {
        /* 底盘模式：左摇杆左右旋转，右摇杆平移 */
        ChassisSetVelocity(
            ApplyDeadzone(RemoteControl::rcInfo.right_col, 0.02f) * CHASSIS_MAX_LINEAR,
            ApplyDeadzone(RemoteControl::rcInfo.right_rol, 0.02f) * CHASSIS_MAX_LINEAR,
            ApplyDeadzone(RemoteControl::rcInfo.left_rol, 0.02f) * CHASSIS_MAX_ROT_RPM);

        /* 推杆停、总线保持角度；齿条仍按角度自动跟随 */
        pushrod_all_off();
        BusServoAngleControl(0.0f, false);
        PlateControl(0.0f, g_busServoAngle);
    } else if (RemoteControl::rcInfo.SA == UP_POS) {
        /* 机构模式：底盘停 */
        ChassisStop();

        /* 左摇杆左右 -> 齿条（手动覆盖 + 自动跟随） */
        PlateControl(ApplyDeadzone(RemoteControl::rcInfo.left_rol, 0.02f), g_busServoAngle);

        /* 左摇杆上下 -> 推杆 */
        PushrodByStick(RemoteControl::rcInfo.left_col);

        /* 右摇杆上下 -> 总线舵机角度 */
        BusServoAngleControl(RemoteControl::rcInfo.right_col, true);
    } else {
        // 未知危险状态
        ChassisStop();
        UserStop();
    }
}
