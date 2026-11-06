//
// Created by Lenovo on 2026/11/6.
//

#include "PushRod.h"


void pushrod_off() {
    HAL_GPIO_WritePin(EXTRA_GPIO_PWM6_GPIO_Port, EXTRA_GPIO_PWM6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EXTRA_GPIO_PWM7_GPIO_Port, EXTRA_GPIO_PWM7_Pin, GPIO_PIN_RESET);
}

void pushrod_out() {
    HAL_GPIO_WritePin(EXTRA_GPIO_PWM6_GPIO_Port, EXTRA_GPIO_PWM6_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(EXTRA_GPIO_PWM7_GPIO_Port, EXTRA_GPIO_PWM7_Pin, GPIO_PIN_SET);
}

void pushrod_in() {
    HAL_GPIO_WritePin(EXTRA_GPIO_PWM6_GPIO_Port, EXTRA_GPIO_PWM6_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EXTRA_GPIO_PWM7_GPIO_Port, EXTRA_GPIO_PWM7_Pin, GPIO_PIN_RESET);
}

void pushrod_toggle() {
    if (!HAL_GPIO_ReadPin(EXTRA_GPIO_PWM6_GPIO_Port, EXTRA_GPIO_PWM6_Pin) &&
        !HAL_GPIO_ReadPin(EXTRA_GPIO_PWM7_GPIO_Port, EXTRA_GPIO_PWM7_Pin)) {
        // 如果处于关闭状态，则转到 out
        pushrod_out();
        }
    // 否则反转方向
    HAL_GPIO_TogglePin(EXTRA_GPIO_PWM6_GPIO_Port, EXTRA_GPIO_PWM6_Pin);
    HAL_GPIO_TogglePin(EXTRA_GPIO_PWM7_GPIO_Port, EXTRA_GPIO_PWM7_Pin);
}
