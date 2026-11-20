//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_DEVICE_H
#define RM_FRAME_C_DEVICE_H
#include "main.h"

#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"


#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "usbd_cdc_if.h"

#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}
/*枚举类型定义------------------------------------------------------------*/

extern void ChassisStart();
extern void ChassisHandle();
extern void CtrlHandle();

/*
 * 设备类型枚举
 */
typedef enum{
    MOTOR,SERVO
}DEVICE_TYPE_E;

/*结构体定义--------------------------------------------------------------*/
typedef struct {
    uint32_t robot_ID;
    uint32_t yaw_zero;
    uint32_t pitch_zero;
    uint32_t busServoCalibMagic;
    int16_t busServoCalibOffset[2];
    int16_t busServoCalibReferenceDeg;
    uint16_t busServoCalibReserved;
    uint32_t pat[125];
}flash_data_t;

#define BUS_SERVO_CALIB_MAGIC 0x4253434Cu   // BSCL

typedef struct {
    float x,y,z;
    float temp;
}ist8310_t;
/*类型定义----------------------------------------------------------------*/

class Device {
protected:
    DEVICE_TYPE_E deviceType;
    uint32_t deviceID;

    virtual void Handle() = 0;

    virtual void ErrorHandle() = 0;

};

/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/
extern flash_data_t flashData;

/*外部函数声明-------------------------------------------------------------*/
bool bsp_flash_write(flash_data_t *_flashData);
void bsp_flash_read(flash_data_t *_flashData);
float bsp_BuzzerOn(float _freq, float _targetVolPct);
void bsp_BuzzerOff();



#endif //RM_FRAME_C_DEVICE_H
