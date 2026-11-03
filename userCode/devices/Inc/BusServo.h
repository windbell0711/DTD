#ifndef RM_FRAME_C_BUS_SERVO_H
#define RM_FRAME_C_BUS_SERVO_H

/**
 * @file    BusServo.h
 * @brief   总线舵机驱动（ST3215 / SCS 协议，单线半双工）
 *
 * 和普通 PWM 舵机不是一回事：
 *   PWM 舵机  = 50Hz 脉宽，单向，无反馈
 *   总线舵机  = 串行数据帧，双向，可读位置/速度/负载/温度
 *
 * ============================== 接线 ==============================
 *   C 板 UART 的 TX 引脚 ──── 舵机信号线（一根）
 *   GND ──────────────────── 舵机 GND（必须共地）
 *   舵机电源 6~12.6V ─────── 独立供电（不要从 C 板取）
 *
 *   总线是单线的：板子发的时候舵机听，舵机回的时候板子听。
 *   驱动初始化时会把 UART 设成半双工（HDSEL），TX 引脚兼作收发。
 *
 *   这里用 USART6，对应 C 板的 UART2（3-pin：5V / GND / 信号），
 *   信号脚是芯片的 PG14（USART6_TX）。
 *
 * ============================== 协议 ==============================
 *   帧：0xFF 0xFF ID LEN INSTR PARAM... CHECKSUM
 *   应答：0xFF 0xFF ID LEN ERR PARAM... CHECKSUM
 *   CHECKSUM = ~(ID + LEN + INSTR + PARAM...) & 0xFF
 */
#include "Device.h"

#define BUS_SERVO_UART      (&huart6)
#define BUS_SERVO_BAUD      1000000u

class BusServo {
public:
    explicit BusServo(UART_HandleTypeDef *huart, uint32_t baud = BUS_SERVO_BAUD);

    /** 检测某个 ID 的舵机是否在线 */
    bool ping(uint8_t id);

    /** 写目标位置（0~4095 一圈），附速度和加速度 */
    bool writePos(uint8_t id, int16_t pos, uint16_t speed = 1500, uint8_t acc = 50);

    /**
     * @brief 同步写：一条帧同时给多个舵机下发目标位置
     * @param ids  舵机 ID 数组
     * @param pos  目标位置数组（与 ids 一一对应）
     * @param n    舵机个数（1~8）
     * @note  同步写是广播帧，舵机不回应答；返回 true 只代表发送成功
     */
    bool syncWritePos(const uint8_t *ids, const int16_t *pos,
                      uint16_t speed = 1500, uint8_t acc = 50, uint8_t n = 2);

    /** 读当前位置（0~4095） */
    bool readPos(uint8_t id, int16_t &pos);

    /** 扭矩开关：false = Release（可手掰），true = 锁住 */
    bool setTorque(uint8_t id, bool enable);

    /**
     * @brief 改舵机 ID（写 EPROM，掉电保存）
     * @note  总线上只能接「一个」舵机时操作；改完后续通信要用新 ID
     */
    bool setID(uint8_t oldId, uint8_t newId);

private:
    bool sendFrame(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t n);
    bool recvAck(uint8_t id, uint8_t *params, uint8_t &n);
    bool writeReg(uint8_t id, uint8_t addr, uint8_t value);

    UART_HandleTypeDef *uart_;
};

#endif // RM_FRAME_C_BUS_SERVO_H
