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
 * 调试开关（默认关闭）：
 * RC_DEBUG 打开后，每 100 ms 通过 USB 串口打印档位原始值/摇杆/四轮转速，
 * 用来确认遥控器档位、电机反馈是否正常。只影响输出，不改变控制逻辑。
 */
#define RC_DEBUG

/*
 * 总线舵机自检：
 * 打开后，ID=1 和 ID=2 两个总线舵机会镜像来回扫动（1000~3000）。
 * 用来验证「USART6 接线 / 波特率 / 半双工 / 舵机 ID」是否正确。
 * 正常遥控控制时保持关闭。
 */
//#define BUS_SERVO_TEST

/*
 * 改舵机 ID（默认关闭）：
 * 总线上只接「一个」舵机时打开，上电会把 ID 1 改成 ID 2。
 * 注意：这个模式和 BUS_SERVO_TEST 不要同时打开。
 */
//#define BUS_SERVO_SET_ID

/*
 * 180° PWM 舵机自检（升降板）：
 * 舵机接 SERVO_ID_3 → TIM1_CH3 → PE13；如果接在第四个口就改成 SERVO_ID_4（PE14）。
 * 打开后舵机在 40°~140° 之间来回摆动。
 */
#define PWM180_TEST
#define PWM180_SERVO_ID     SERVO_ID_3

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

/*外部函数声明-------------------------------------------------------------*/

void UserStop();
void UserHandle();
void UserInit();
void BusServoSetPosition(SWITCH_STATE_E sw);
#endif //RM_FRAME_C_USERTASK_H
