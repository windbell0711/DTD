//
// Created by Lenovo on 2026/11/6.
//

#include "PushRod.h"

namespace {

void _set_rod_1(const bool extend) {
    if (extend) {
        HAL_GPIO_WritePin(RELAY_Port_1, RELAY_Pin_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RELAY_Port_2, RELAY_Pin_2, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(RELAY_Port_2, RELAY_Pin_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RELAY_Port_1, RELAY_Pin_1, GPIO_PIN_SET);
    }
}

void _set_rod_2(const bool extend) {
    if (extend) {
        HAL_GPIO_WritePin(RELAY_Port_3, RELAY_Pin_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RELAY_Port_4, RELAY_Pin_4, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(RELAY_Port_4, RELAY_Pin_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RELAY_Port_3, RELAY_Pin_3, GPIO_PIN_SET);
    }
}

} // namespace

void pushrod_1_off() {
    HAL_GPIO_WritePin(RELAY_Port_1, RELAY_Pin_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_Port_2, RELAY_Pin_2, GPIO_PIN_RESET);
}

void pushrod_2_off() {
    HAL_GPIO_WritePin(RELAY_Port_3, RELAY_Pin_3, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_Port_4, RELAY_Pin_4, GPIO_PIN_RESET);
}

void pushrod_1_out() {
    _set_rod_1(true);
}

void pushrod_2_out() {
    _set_rod_2(true);
}

void pushrod_1_in() {
    _set_rod_1(false);
}

void pushrod_2_in() {
    _set_rod_2(false);
}

void pushrod_all_off() {
    pushrod_1_off();
    pushrod_2_off();
}

void pushrod_all_out() {
    pushrod_1_out();
    pushrod_2_out();
}

void pushrod_all_in() {
    pushrod_1_in();
    pushrod_2_in();
}
