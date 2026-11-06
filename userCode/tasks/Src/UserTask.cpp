//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"
#include "ChassisTask.h"
#include "PushRod.h"
#include "BusServo.h"

static BusServo *g_busServo = nullptr;
static int8_t g_busServoPreset = -1;
static bool g_busServoTorqueEnabled = false;
static uint32_t g_busServoLastRefreshTick = 0;
static uint32_t g_busServoLastTorqueOffTick = 0;

namespace {
constexpr uint8_t kBusServoCount = 2;
constexpr uint8_t kBusServoIds[kBusServoCount] = {BUS_SERVO_ID_1, BUS_SERVO_ID_2};
constexpr uint16_t kBusServoSpeed = 600;
constexpr uint8_t kBusServoAcceleration = 30;
constexpr uint32_t kBusServoRefreshMs = 50;
constexpr uint32_t kBusServoTorqueOffRetryMs = 100;

bool BusServoWriteTorque(bool enable) {
    if (g_busServo == nullptr) {
        return false;
    }
    return g_busServo->syncWriteTorque(kBusServoIds, enable, kBusServoCount);
}

bool BusServoWritePreset(int8_t preset) {
    if ((g_busServo == nullptr) || (preset < 0) || (preset > 2)) {
        return false;
    }

    const int16_t presetPos[3] = {
        BUS_SERVO_POS_UP,
        BUS_SERVO_POS_MID,
        BUS_SERVO_POS_DOWN
    };

    int16_t pos[kBusServoCount];
    pos[0] = presetPos[preset];
#if BUS_SERVO_MIRROR_SECOND
    pos[1] = (int16_t)(4095 - pos[0]);
#else
    pos[1] = pos[0];
#endif

    return g_busServo->syncWritePos(kBusServoIds, pos, kBusServoSpeed,
                                    kBusServoAcceleration, kBusServoCount);
}
}

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

    if (g_busServo != nullptr) {
        const uint32_t now = HAL_GetTick();
        if (g_busServoTorqueEnabled ||
            ((now - g_busServoLastTorqueOffTick) >= kBusServoTorqueOffRetryMs)) {
            BusServoWriteTorque(false);
            g_busServoTorqueEnabled = false;
            g_busServoLastTorqueOffTick = now;
        }
    }
    g_busServoPreset = -1;

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

    // UART 已初始化，构造总线舵机对象
    static BusServo servo(BUS_SERVO_UART);
    g_busServo = &servo;
    g_busServoLastTorqueOffTick = HAL_GetTick() - kBusServoTorqueOffRetryMs;
}

/**
 * @brief 按遥控器 SC 三挡设置总线舵机目标位置
 * @note  档位变化立即下发，并每 50 ms 重发一次，丢包或舵机重新上电后可恢复
 */
void BusServoSetPosition(SWITCH_STATE_E sw) {
    int8_t preset;
    switch (sw) {
        case UP_POS:   preset = 0; break;
        case MID_POS:  preset = 1; break;
        case DOWN_POS: preset = 2; break;
        default:       return;
    }

    const uint32_t now = HAL_GetTick();
    if ((preset == g_busServoPreset) &&
        ((now - g_busServoLastRefreshTick) < kBusServoRefreshMs)) {
        return;
    }

    if (BusServoWriteTorque(true)) {
        g_busServoTorqueEnabled = true;
    }
    BusServoWritePreset(preset);
    g_busServoPreset = preset;
    g_busServoLastRefreshTick = now;
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
}
