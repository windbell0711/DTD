//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_USERTASK_H
#define RM_FRAME_C_USERTASK_H
#include "main.h"
#include "Device.h"
#include "Servo.h"
#include "Motor.h"
#include "RemoteControl.h"
/*枚举类型定义------------------------------------------------------------*/


/*结构体定义--------------------------------------------------------------*/


/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/

/*
 * 总线舵机控制：
 *   ID2 默认与 ID1 镜像安装，目标位置自动取 4095 - ID1 的目标位置。
 *   角度限位 55~180 度，BusServoSetAngle() 内部 clamp。
 *   SC 上/中时左摇杆上下直接改变目标角度，变化率 = 摇杆值 * ANGLE_RATE_MAX。
 */
#define BUS_SERVO_ID_1              1
#define BUS_SERVO_ID_2              2
#define BUS_SERVO_MIRROR_SECOND     1
#define BUS_SERVO_ANGLE_MIN         10.0f
#define BUS_SERVO_ANGLE_MAX         140.0f
#define BUS_SERVO_ANGLE_INIT        90.0f    // 上电/首次进入控制时的目标角度
#define BUS_SERVO_ANGLE_RATE_MAX    50.0f    // 度/秒，满杆时的角度变化率

extern float g_busServoAngle;

/* ---------- plate (rack) servo parameters ---------- */
#define PLATE_SERVO_ID              SERVO_ID_5   /* PWM5 -> PC6 (TIM8_CH1) */
#define PLATE_SPEED                 90.0f
#define PLATE_MOVE_MS               4500u        /* full travel time, measure and tune */
#define PLATE_DIR_EXTEND            (+1)         /* swap with RETRACT if direction reversed */
#define PLATE_DIR_RETRACT           (-1)
#define PLATE_HOME_SPEED            90.0f
#define PLATE_HOME_MS               (PLATE_MOVE_MS)
/* ---------- plate auto-follow / manual override ---------- */
/* 同步带速度（百分比），main 里已有同名宏时以 main 为准 */
#ifndef SERVO_FRONT_SPEED_PERC
#define SERVO_FRONT_SPEED_PERC  90
#endif
#ifndef SERVO_REAR_SPEED_PERC
#define SERVO_REAR_SPEED_PERC   90
#endif

#define PLATE_AUTO_THRESHOLD_DEG    80.0f    /* 总线角度高于此值 -> 齿条伸出 */
#define PLATE_AUTO_HYST_DEG          2.0f    /* 阈值死区，防止抖动 */
#define PLATE_MANUAL_DEADZONE        0.4f    /* 手动摇杆死区 */
#define PLATE_MANUAL_SPEED          60.0f    /* 手动速度（比自动慢，便于微调） */
#define PLATE_MANUAL_DIR_SIGN       (+1.0f)  /* 摇杆向右 = 伸出；反了就改 -1 */
#define PLATE_PUSHROD_THRESHOLD      0.5f    /* 推杆摇杆阈值 */
#define PLATE_POS_LOW_MS            0u
#define PLATE_POS_MID_MS            (PLATE_MOVE_MS / 2u)
#define PLATE_POS_HIGH_MS           (PLATE_MOVE_MS)

extern Servo ClawServo,TurnLServo,TurnRServo,ShovelLServo,ShovelRServo;

typedef enum {
    BUS_SERVO_LINK_NONE = 0,    // 总线舵机全部无回包
    BUS_SERVO_LINK_PARTIAL,     // 部分舵机回包
    BUS_SERVO_LINK_ALL          // 全部舵机在线
} BUS_SERVO_LINK_STATE_E;

/*外部函数声明-------------------------------------------------------------*/

void UserStop();
void UserHandle();
void UserInit();
void BusServoSetAngle(float angle);
void PlateControl(float manualStick, float busAngle);
void BusServoAngleControl(float stickUpDown, bool enable);
bool BusServoCalibrationFeedbackColor(uint32_t *color);
bool BusServoFaultColor(uint32_t *color);
BUS_SERVO_LINK_STATE_E BusServoLinkState();
#endif //RM_FRAME_C_USERTASK_H
