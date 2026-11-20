#include "BusServo.h"

namespace {
constexpr uint8_t kInstrPing  = 0x01;
constexpr uint8_t kInstrRead  = 0x02;
constexpr uint8_t kInstrWrite = 0x03;

constexpr uint8_t kAddrTorque = 0x28;   // 扭矩开关
constexpr uint8_t kAddrPos    = 0x38;   // 当前位置
constexpr uint8_t kAddrAcc    = 0x29;   // 加速度（后面紧跟目标位置/运行时间/运行速度）

// 等待应答的轮询次数上限；收到完整回包会立即退出
constexpr uint32_t kRecvSpin = 20000u;
}

BusServo::BusServo(UART_HandleTypeDef *huart, uint32_t baud) : uart_(huart) {
    uart_->Init.BaudRate     = baud;
    uart_->Init.WordLength   = UART_WORDLENGTH_8B;
    uart_->Init.StopBits     = UART_STOPBITS_1;
    uart_->Init.Parity       = UART_PARITY_NONE;
    uart_->Init.Mode         = UART_MODE_TX_RX;
    uart_->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    uart_->Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_HalfDuplex_Init(uart_);

    // 单线半双工：TX 引脚兼作收发，空闲时释放总线
    // HDSEL is set by HAL_HalfDuplex_Init().
    // 常开接收，发的时候会临时关掉
    HAL_HalfDuplex_EnableReceiver(uart_);
}

bool BusServo::startRxDma() {
    DMA_HandleTypeDef *dma = uart_->hdmarx;
    if ((dma == nullptr) || (dma->Instance == nullptr)) {
        return false;
    }

    // 发送期间关闭接收，避免总线上出现本机发送的帧
    CLEAR_BIT(uart_->Instance->CR1, USART_CR1_RE);
    __HAL_UART_CLEAR_OREFLAG(uart_);

    __HAL_DMA_DISABLE(dma);
    uint32_t guard = kRecvSpin;
    while ((dma->Instance->CR & DMA_SxCR_EN) != 0U) {
        if (--guard == 0U) {
            return false;
        }
    }

    dma->Instance->PAR = (uint32_t)&(uart_->Instance->DR);
    dma->Instance->M0AR = (uint32_t)rxBuf_;
    dma->Instance->NDTR = sizeof(rxBuf_);

    // Clear stale DMA2 Stream2 flags before rearming.
    DMA2->LIFCR = 0x003D0000u;
    SET_BIT(uart_->Instance->CR3, USART_CR3_DMAR);
    __HAL_DMA_ENABLE(dma);
    return ((dma->Instance->CR & DMA_SxCR_EN) != 0U);
}

void BusServo::stopRxDma() {
    DMA_HandleTypeDef *dma = uart_->hdmarx;

    CLEAR_BIT(uart_->Instance->CR3, USART_CR3_DMAR);
    CLEAR_BIT(uart_->Instance->CR1, USART_CR1_RE);
    if ((dma == nullptr) || (dma->Instance == nullptr)) {
        return;
    }

    __HAL_DMA_DISABLE(dma);
    uint32_t guard = kRecvSpin;
    while ((dma->Instance->CR & DMA_SxCR_EN) != 0U) {
        if (--guard == 0U) {
            break;
        }
    }
}

uint8_t BusServo::rxDmaCount() const {
    if ((uart_->hdmarx == nullptr) || (uart_->hdmarx->Instance == nullptr)) {
        return 0;
    }

    const uint32_t remain = uart_->hdmarx->Instance->NDTR;
    if (remain > sizeof(rxBuf_)) {
        return (uint8_t)sizeof(rxBuf_);
    }
    return (uint8_t)(sizeof(rxBuf_) - remain);
}

bool BusServo::parseAckFrame(uint8_t id, const volatile uint8_t *buf, uint8_t count,
                             uint8_t *params, uint8_t &n, uint8_t maxParams,
                             bool &servoError) const {
    n = 0;
    servoError = false;

    // 在 DMA 缓冲区中扫描完整帧，前面的干扰字节不影响后续重新同步
    for (uint8_t start = 0; (uint8_t)(start + 4u) <= count; ++start) {
        if ((buf[start] != 0xFF) || (buf[start + 1u] != 0xFF)) {
            continue;
        }

        const uint8_t len = buf[start + 3u];
        if (len < 2u) {
            continue;
        }

        const uint16_t total = (uint16_t)len + 4u;
        const uint8_t replyId = buf[start + 2u];
        if ((total > count) || (replyId != id)) {
            continue;
        }

        uint16_t sum = 0;
        for (uint16_t k = 0; k < (uint16_t)(len + 1u); ++k) {
            sum += buf[start + 2u + k];
        }
        if ((uint8_t)(~sum) != buf[start + total - 1u]) {
            continue;
        }

        const uint8_t dataLen = (uint8_t)(len - 2u);
        if (dataLen > maxParams) {
            continue;
        }

        servoError = (buf[start + 4u] != 0U);
        for (uint8_t k = 0; k < dataLen; ++k) {
            params[k] = buf[start + 5u + k];
        }
        n = dataLen;
        return true;
    }

    return false;
}

bool BusServo::sendFrame(uint8_t id, uint8_t instr, const uint8_t *params, uint8_t n,
                         bool expectAck) {
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

    // DMA 提前挂好，发送结束后立即切接收，避免漏掉应答首字节
    if (!startRxDma()) {
        return false;
    }

    SET_BIT(uart_->Instance->CR1, USART_CR1_TE);
    HAL_StatusTypeDef st = HAL_UART_Transmit(uart_, frame, i, 2);
    CLEAR_BIT(uart_->Instance->CR1, USART_CR1_TE);
    SET_BIT(uart_->Instance->CR1, USART_CR1_RE);

    const bool ok = (st == HAL_OK);
    if (!ok) {
        // HAL_UART_Transmit 超时后不会自动恢复 gState
        uart_->gState = HAL_UART_STATE_READY;
    }
    if (!ok || !expectAck) {
        stopRxDma();
    }

    return ok;
}

bool BusServo::recvAck(uint8_t id, uint8_t *params, uint8_t &n, uint8_t maxParams,
                       bool *servoError) {
    uint32_t guard = kRecvSpin;

    n = 0;
    if (servoError != nullptr) {
        *servoError = false;
    }
    // 先等第一个字节，避免无应答时把完整超时都耗在逐帧解析上
    while (guard-- > 0U) {
        if (rxDmaCount() != 0U) {
            break;
        }
    }
    if (rxDmaCount() == 0U) {
        stopRxDma();
        return false;
    }

    guard = kRecvSpin;
    bool err = false;
    while (guard-- > 0U) {
        const uint8_t count = rxDmaCount();
        if (parseAckFrame(id, rxBuf_, count, params, n, maxParams, err)) {
            if (servoError != nullptr) {
                *servoError = err;
            }
            stopRxDma();
            return true;
        }
        if (count >= sizeof(rxBuf_)) {
            break;
        }
    }

    stopRxDma();
    return false;
}

bool BusServo::ping(uint8_t id) {
    uint8_t params[4];
    uint8_t n = 0;
    bool err = false;
    if (!sendFrame(id, kInstrPing, nullptr, 0)) {
        return false;
    }
    // 只要回包合法就说明舵机在线；ERR 非 0 属于舵机自身状态，不算掉线
    return recvAck(id, params, n, sizeof(params), &err);
}

void BusServo::setBaud(uint32_t baud) {
    uart_->Init.BaudRate = baud;
    if (HAL_HalfDuplex_Init(uart_) != HAL_OK) {
        return;
    }
    (void)HAL_HalfDuplex_EnableReceiver(uart_);
    __HAL_UART_CLEAR_OREFLAG(uart_);
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
    bool err = false;
    if (!sendFrame(id, kInstrWrite, p, sizeof(p))) {
        return false;
    }
    return recvAck(id, params, n, sizeof(params), &err) && !err;
}

bool BusServo::readRegs(uint8_t id, uint8_t addr, uint8_t cnt, uint8_t *out,
                        bool *servoError) {
    if ((out == nullptr) || (cnt == 0u) || (cnt > 8u)) {
        return false;
    }

    const uint8_t p[2] = {addr, cnt};
    uint8_t params[10];
    uint8_t n = 0;
    bool err = false;

    if (!sendFrame(id, kInstrRead, p, sizeof(p))) {
        return false;
    }
    if (!recvAck(id, params, n, sizeof(params), &err) || (n < cnt)) {
        return false;
    }

    for (uint8_t i = 0; i < cnt; ++i) {
        out[i] = params[i];
    }
    if (servoError != nullptr) {
        *servoError = err;
    }
    return true;
}
bool BusServo::readPos(uint8_t id, int16_t &pos) {
    const uint8_t p[2] = {kAddrPos, 2};

    uint8_t params[4];
    uint8_t n = 0;
    bool err = false;
    if (!sendFrame(id, kInstrRead, p, sizeof(p))) {
        return false;
    }
    if (!recvAck(id, params, n, sizeof(params), &err) || err || (n < 2)) {
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
    bool err = false;
    if (!sendFrame(id, kInstrWrite, p, sizeof(p))) {
        return false;
    }
    return recvAck(id, params, n, sizeof(params), &err) && !err;
}

bool BusServo::writeReg(uint8_t id, uint8_t addr, uint8_t value) {
    uint8_t p[2];
    p[0] = addr;
    p[1] = value;

    uint8_t params[4];
    uint8_t n = 0;
    bool err = false;
    if (!sendFrame(id, kInstrWrite, p, sizeof(p))) {
        return false;
    }
    return recvAck(id, params, n, sizeof(params), &err) && !err;
}

bool BusServo::writeEepromReg(uint8_t id, uint8_t addr, uint8_t value) {
    constexpr uint8_t kAddrLock = 0x37u;   /* EPROM 写保护：0=解锁, 1=上锁 */

    (void)setTorque(id, false);            /* EPROM 写入前必须关扭矩 */
    if (!writeReg(id, kAddrLock, 0u)) {
        return false;
    }

    const bool ok = writeReg(id, addr, value);
    (void)writeReg(id, kAddrLock, 1u);
    return ok;
}
bool BusServo::setID(uint8_t oldId, uint8_t newId) {
    // 解锁 EPROM（0x37 写 0），否则改完掉电会丢
    if (!writeReg(oldId, 0x37, 0)) {
        return false;
    }
    // 写新 ID（0x05）
    // The ACK for the ID write may already carry the NEW ID (the servo applies
    // the register before replying), so do not wait for an ACK under oldId.
    const uint8_t p[2] = {0x05u, newId};
    if (!sendFrame(oldId, kInstrWrite, p, sizeof(p), false)) {
        return false;
    }

    HAL_Delay(50);
    __HAL_UART_CLEAR_OREFLAG(uart_);

    bool newIdOk = false;
    for (uint8_t attempt = 0; attempt < 3u; ++attempt) {
        if (ping(newId)) {
            newIdOk = true;
            break;
        }
        HAL_Delay(10);
    }
    if (!newIdOk) {
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
    return sendFrame(0xFE, 0x83, p, i, false);
}

bool BusServo::syncWriteTorque(const uint8_t *ids, bool enable, uint8_t n) {
    if ((n == 0) || (n > 8) || (ids == nullptr)) {
        return false;
    }

    uint8_t p[2 + 8 * 2];
    uint8_t i = 0;

    p[i++] = kAddrTorque;
    p[i++] = 1;

    for (uint8_t k = 0; k < n; ++k) {
        p[i++] = ids[k];
        p[i++] = enable ? 1 : 0;
    }

    return sendFrame(0xFE, 0x83, p, i, false);
}
