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
 * 总线舵机遥控映射（SA 下档时由 SC 三挡控制）：
 *   SC 上 / 中 / 下 -> BUS_SERVO_POS_UP / MID / DOWN
 * 位置为 ST3215 原始值，0~4095 对应一圈；先按机械行程改这三个值。
 * ID2 默认与 ID1 镜像安装，目标位置自动取 4095 - ID1 的目标位置。
 */
#define BUS_SERVO_ID_1              1
#define BUS_SERVO_ID_2              2
#define BUS_SERVO_MIRROR_SECOND     1
#define BUS_SERVO_POS_UP            1000
#define BUS_SERVO_POS_MID           2048
#define BUS_SERVO_POS_DOWN          3000

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
void BusServoSetPosition(SWITCH_STATE_E sw);
BUS_SERVO_LINK_STATE_E BusServoLinkState();
#endif //RM_FRAME_C_USERTASK_H
