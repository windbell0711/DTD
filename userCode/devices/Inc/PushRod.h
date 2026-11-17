//
// Created by Lenovo on 2026/11/6.
//

#ifndef RM_FRAME_C_PUSHROD_H
#define RM_FRAME_C_PUSHROD_H

#include "main.h"

#define RELAY_Port_1 (EXTRA_PWM6_GPIO_Port)
#define RELAY_Pin_1  (EXTRA_PWM6_Pin)

#define RELAY_Port_2 (EXTRA_PWM7_GPIO_Port)
#define RELAY_Pin_2  (EXTRA_PWM7_Pin)

#define RELAY_Port_3 (EXTRA_PF0_GPIO_Port)
#define RELAY_Pin_3  (EXTRA_PF0_Pin)

#define RELAY_Port_4 (EXTRA_PF1_GPIO_Port)
#define RELAY_Pin_4  (EXTRA_PF1_Pin)

void pushrod_1_off();
void pushrod_2_off();

void pushrod_1_out();
void pushrod_2_out();

void pushrod_1_in();
void pushrod_2_in();

/* 两根推杆同动接口 */
void pushrod_all_off();
void pushrod_all_out();
void pushrod_all_in();

#endif //RM_FRAME_C_PUSHROD_H
