//
// Created by LEGION on 2021/10/10.
//

#ifndef RM_FRAME_C_REMOTECONTROL_H
#define RM_FRAME_C_REMOTECONTROL_H

#include "Device.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 25u
#define RC_CH_VALUE_OFFSET 1024
#define RC_LINK_TIMEOUT_MS 100u
/*枚举类型定义------------------------------------------------------------*/
typedef enum{
    UP_POS = 97u,MID_POS = 0u,DOWN_POS = 158u
}SWITCH_STATE_E;

/*
 * 三位开关(SB/SC)的原始通道值（S.BUS 11 位，未减偏移）。
 * 这几个数必须与手头遥控器的实际读数一致，否则档位永远判不对。
 * 当前值：2026-10 实测天地飞 ET08 + W.BUS，SB 三档为 353 / 1024 / 1694。
 * 换遥控器/换接收机后请重新测：打开 RC_DEBUG，拨动 SB/SC，看 s[0]/s[1]。
 * 判档逻辑见 RemoteControl.cpp 的 classifySwitch()。
 */
#define RC_SW_UP_RAW    353
#define RC_SW_MID_RAW   1024
#define RC_SW_DOWN_RAW  1694

/*
 * 摇杆中位校准：松开摇杆、手离开时各通道的残留偏移（原始值，已减 1024 之后）。
 * 2026-10 实测 ET08：ch0 = -40、ch3 = -8，ch1/ch2 为 0。
 * 不校准的话，推前进会附带一个固定方向的平移（车会慢慢漂）。
 * 换遥控器后重新测：松手看 rc_ctrl.rc.ch[0..3]，把数值填到这里。
 */
#define RC_TRIM_CH0     (-40)
#define RC_TRIM_CH1     (0)
#define RC_TRIM_CH2     (0)
#define RC_TRIM_CH3     (-8)
/*结构体定义--------------------------------------------------------------*/
typedef struct {
    int32_t ch[4];
    uint32_t s[2];
    int16_t optional[2];
}rc_t;

typedef struct {
    int32_t x,y,z,press_l,press_r;
}mouse_t;

typedef struct {
    uint32_t v;
}keyboard_t;

typedef struct {
    rc_t rc;
    mouse_t mouse;
    keyboard_t key;
}RC_ctrl_t;

typedef struct {
    float left_col,left_rol;
    float right_col,right_rol;
    SWITCH_STATE_E SB;
    uint32_t pat;
    SWITCH_STATE_E SC;
    int16_t optional[2];
    SWITCH_STATE_E SA;
    SWITCH_STATE_E SD;
} RC_Info_t;

/*类型定义----------------------------------------------------------------*/
class RemoteControl : public Device {
public:
    static RC_Info_t rcInfo;
    static RC_ctrl_t rc_ctrl;

    static uint32_t lastFrameTick;   //最近一次收到有效遥控帧的时刻
    static bool isOnline();          //遥控器是否在线（超时判据见 RemoteControl.cpp）

    static void init();
    static void ITHandle();
    static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
    static void sbus_to_rc(volatile const uint8_t *sbus_buf);

private:
    static volatile uint32_t lastUpdateTick;
    static volatile bool online;
};

/*结构体成员取值定义组------------------------------------------------------*/

/*外部变量声明-------------------------------------------------------------*/

/*外部函数声明-------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
extern void USART3_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_REMOTECONTROL_H
