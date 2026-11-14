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
#define BUS_SERVO_ANGLE_MIN         57.2f
#define BUS_SERVO_ANGLE_MAX         180.0f
#define BUS_SERVO_ANGLE_INIT        90.0f    // 上电/首次进入控制时的目标角度
#define BUS_SERVO_ANGLE_RATE_MAX    60.0f    // 度/秒，满杆时的角度变化率

extern float g_busServoAngle;

/* ---------- plate (rack) servo parameters ---------- */
#define PLATE_SERVO_ID              SERVO_ID_5   /* PWM5 -> PC6 (TIM8_CH1) */
#define PLATE_SPEED                 90.0f
#define PLATE_MOVE_MS               4500u        /* full travel time, measure and tune */
#define PLATE_DIR_EXTEND            (+1)         /* swap with RETRACT if direction reversed */
#define PLATE_DIR_RETRACT           (-1)
#define PLATE_HOME_SPEED            90.0f
#define PLATE_HOME_MS               (PLATE_MOVE_MS)
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
void PlateSetSwitch(SWITCH_STATE_E sw);
void BusServoAngleControl(float stickUpDown, bool enable);
bool BusServoCalibrationFeedbackColor(uint32_t *color);
BUS_SERVO_LINK_STATE_E BusServoLinkState();
#endif //RM_FRAME_C_USERTASK_H
