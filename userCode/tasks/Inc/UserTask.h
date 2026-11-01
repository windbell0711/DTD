//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_USERTASK_H
#define RM_FRAME_C_USERTASK_H
#include "main.h"
#include "Device.h"
#include "Servo.h"
#include "Motor.h"
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
 * 用来验证「USART1 接线 / 波特率 / 半双工 / 舵机 ID」是否正确。
 */
#define BUS_SERVO_TEST

/*
 * 改舵机 ID（默认关闭）：
 * 总线上只接「一个」舵机时打开，上电会把 ID 1 改成 ID 2。
 * 注意：这个模式和 BUS_SERVO_TEST 不要同时打开。
 */
//#define BUS_SERVO_SET_ID

extern Servo ClawServo,TurnLServo,TurnRServo,ShovelLServo,ShovelRServo;

/*外部函数声明-------------------------------------------------------------*/

void UserStop();
void UserHandle();
void UserInit();
#endif //RM_FRAME_C_USERTASK_H
