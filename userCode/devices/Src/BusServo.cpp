#include "BusServo.h"

namespace {
constexpr uint8_t kInstrPing  = 0x01;
constexpr uint8_t kInstrRead  = 0x02;
constexpr uint8_t kInstrWrite = 0x03;

constexpr uint8_t kAddrTorque = 0x28;   // 扭矩开关
constexpr uint8_t kAddrPos    = 0x38;   // 当前位置
constexpr uint8_t kAddrAcc    = 0x29;   // 加速度（后面紧跟目标位置/运行时间/运行速度）

// 等待应答的轮询次数，约 0.5 ms（168 MHz 下）
constexpr uint32_t kRecvSpin = 12000u;

/**
 * @brief 补上 USART6 的引脚复用配置
 * @note  CubeMX 生成的 HAL_UART_MspInit 里只有 USART1/USART3 的分支，
 *        USART6 的 PG9/PG14 从来没有被配成 AF8，串口寄存器在动但引脚不出波形。
 */
void ensureUsart6Pins() {
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_14;          // USART6_TX，半双工下兼作收发（板子 UART2 口）
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &gpio);

    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_7;                 // USART6_RX（板子 UART2 口）
    HAL_GPIO_Init(GPIOC, &gpio);
}
}

BusServo::BusServo(UART_HandleTypeDef *huart, uint32_t baud) : uart_(huart) {
    ensureUsart6Pins();

    uart_->Init.BaudRate     = baud;
    uart_->Init.WordLength   = UART_WORDLENGTH_8B;
    uart_->Init.StopBits     = UART_STOPBITS_1;
    uart_->Init.Parity       = UART_PARITY_NONE;
    uart_->Init.Mode         = UART_MODE_TX_RX;
    uart_->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    uart_->Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(uart_);

    // 单线半双工：TX 引脚兼作收发，空闲时释放总线
    SET_BIT(uart_->Instance->CR3, USART_CR3_HDSEL);
    // 常开接收，发的时候会临时关掉
    SET_BIT(uart_->Instance->CR1, USART_CR1_RE);
}

bool BusServo::sendFrame(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t n) {
    uint8_t frame[80];
    uint8_t i = 0;

    if ((uint16_t)n + 6u > sizeof(frame)) {
        return false;
    }

    frame[i++] = 0xFF;
    frame[i++] = 0xFF;
    frame[i++] = id;
    frame[i++] = (uint8_t)(n + 2);
    frame[i++] = instr;
    for (uint8_t k = 0; k < n; ++k) {
        frame[i++] = params[k];
    }

    uint16_t sum = 0;
    for (uint8_t k = 2; k < i; ++k) {
        sum += frame[k];
    }
    frame[i++] = (uint8_t)(~sum);

    // 发送期间关接收，避免把自己发的帧当成应答
    CLEAR_BIT(uart_->Instance->CR1, USART_CR1_RE);
    HAL_StatusTypeDef st = HAL_UART_Transmit(uart_, frame, i, 2);
    SET_BIT(uart_->Instance->CR1, USART_CR1_RE);

    return (st == HAL_OK);
}

bool BusServo::recvAck(uint8_t id, uint8_t *params, uint8_t &n) {
    uint8_t buf[16];
    uint8_t cnt = 0;
    uint32_t guard = kRecvSpin;

    n = 0;

    while (guard-- > 0) {
        if (__HAL_UART_GET_FLAG(uart_, UART_FLAG_RXNE) != RESET) {
            uint8_t b = (uint8_t)(uart_->Instance->DR & 0xFFU);
            guard = kRecvSpin;   // 收到字节就重置超时

            if (cnt == 0 && b != 0xFF) {
                continue;
            }
            if (cnt == 1 && b != 0xFF) {
                cnt = 0;
                continue;
            }
            if (cnt >= sizeof(buf)) {
                return false;
            }
            buf[cnt++] = b;

            if (cnt >= 4) {
                uint8_t len = buf[3];
                if (len < 2) {
                    return false;
                }
                if (cnt >= (uint8_t)(len + 4)) {
                    break;
                }
            }
        } else if (__HAL_UART_GET_FLAG(uart_, UART_FLAG_ORE) != RESET) {
            __HAL_UART_CLEAR_OREFLAG(uart_);
        }
    }

    if (cnt < 4) {
        return false;
    }

    const uint8_t len = buf[3];
    if (cnt < (uint8_t)(len + 4)) {
        return false;
    }
    if (buf[2] != id) {
        return false;
    }

    uint16_t sum = 0;
    for (uint8_t k = 2; k < (uint8_t)(len + 3); ++k) {
        sum += buf[k];
    }
    if ((uint8_t)(~sum) != buf[len + 3]) {
        return false;
    }

    if (buf[4] != 0) {   // ERR 非 0：舵机报错（过载/过温等）
        return false;
    }

    n = (uint8_t)(len - 2);
    for (uint8_t k = 0; k < n; ++k) {
        params[k] = buf[5 + k];
    }
    return true;
}

bool BusServo::ping(uint8_t id) {
    uint8_t params[4];
    uint8_t n = 0;
    if (!sendFrame(id, kInstrPing, nullptr, 0)) {
        return false;
    }
    return recvAck(id, params, n);
}

bool BusServo::writePos(uint8_t id, int16_t pos, uint16_t speed, uint8_t acc) {
    uint8_t p[8];
    p[0] = kAddrAcc;
    p[1] = acc;
    p[2] = (uint8_t)(pos & 0xFF);
    p[3] = (uint8_t)((pos >> 8) & 0xFF);
    p[4] = 0;                                   // 运行时间 = 0（用速度控制）
    p[5] = 0;
    p[6] = (uint8_t)(speed & 0xFF);
    p[7] = (uint8_t)((speed >> 8) & 0xFF);

    uint8_t params[4];
    uint8_t n = 0;
    if (!sendFrame(id, kInstrWrite, p, sizeof(p))) {
        return false;
    }
    return recvAck(id, params, n);
}

bool BusServo::readPos(uint8_t id, int16_t &pos) {
    const uint8_t p[2] = {kAddrPos, 2};

    uint8_t params[4];
    uint8_t n = 0;
    if (!sendFrame(id, kInstrRead, p, sizeof(p))) {
        return false;
    }
    if (!recvAck(id, params, n) || n < 2) {
        return false;
    }

    pos = (int16_t)((uint16_t)params[0] | ((uint16_t)params[1] << 8));
    return true;
}

bool BusServo::setTorque(uint8_t id, bool enable) {
    uint8_t p[2];
    p[0] = kAddrTorque;
    p[1] = enable ? 1 : 0;

    uint8_t params[4];
    uint8_t n = 0;
    if (!sendFrame(id, kInstrWrite, p, sizeof(p))) {
        return false;
    }
    return recvAck(id, params, n);
}

bool BusServo::writeReg(uint8_t id, uint8_t addr, uint8_t value) {
    uint8_t p[2];
    p[0] = addr;
    p[1] = value;

    uint8_t params[4];
    uint8_t n = 0;
    if (!sendFrame(id, kInstrWrite, p, sizeof(p))) {
        return false;
    }
    return recvAck(id, params, n);
}

bool BusServo::setID(uint8_t oldId, uint8_t newId) {
    // 解锁 EPROM（0x37 写 0），否则改完掉电会丢
    if (!writeReg(oldId, 0x37, 0)) {
        return false;
    }
    // 写新 ID（0x05）
    if (!writeReg(oldId, 0x05, newId)) {
        return false;
    }
    // 上锁（0x37 写 1）。此时舵机已经用新 ID 应答
    return writeReg(newId, 0x37, 1);
}

bool BusServo::syncWritePos(const uint8_t *ids, const int16_t *pos,
                            uint16_t speed, uint8_t acc, uint8_t n) {
    if ((n == 0) || (n > 8) || (ids == nullptr) || (pos == nullptr)) {
        return false;
    }

    uint8_t p[2 + 8 * 8];
    uint8_t i = 0;

    p[i++] = kAddrAcc;   // 0x29：加速度/目标位置/运行时间/运行速度 是连续的
    p[i++] = 7;          // 每个舵机的数据长度

    for (uint8_t k = 0; k < n; ++k) {
        p[i++] = ids[k];
        p[i++] = acc;
        p[i++] = (uint8_t)(pos[k] & 0xFF);
        p[i++] = (uint8_t)((pos[k] >> 8) & 0xFF);
        p[i++] = 0;      // 运行时间 0：按速度走
        p[i++] = 0;
        p[i++] = (uint8_t)(speed & 0xFF);
        p[i++] = (uint8_t)((speed >> 8) & 0xFF);
    }

    // 0xFE = 广播 ID，0x83 = SYNC WRITE，舵机不回应答
    return sendFrame(0xFE, 0x83, p, i);
}
