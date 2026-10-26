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

extern Servo ClawServo,TurnLServo,TurnRServo,ShovelLServo,ShovelRServo;

/*外部函数声明-------------------------------------------------------------*/

void UserStop();
void UserHandle();
void UserInit();
#endif //RM_FRAME_C_USERTASK_H
