//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"
#include "ChassisTask.h"
#include "PushRod.h"
#include "BusServo.h"

// Keil Watch 调试变量（方向编码：0中 1上 2右上 3右 4右下 5下 6左下 7左 8左上）
volatile float g_dbgLeftCol = 0.0f;
volatile float g_dbgLeftRol = 0.0f;
volatile float g_dbgRightCol = 0.0f;
volatile float g_dbgRightRol = 0.0f;
volatile uint8_t g_dbgLeftDir = 0;
volatile uint8_t g_dbgRightDir = 0;
volatile uint16_t g_dbgSaRaw = 0;
volatile uint16_t g_dbgSdRaw = 0;
volatile uint16_t g_dbgSbRaw = 0;
volatile uint16_t g_dbgScRaw = 0;
volatile uint8_t g_dbgSaState = 0;       // 0上 1中 2下
volatile uint8_t g_dbgSdState = 0;
volatile uint8_t g_dbgSbState = 0;
volatile uint8_t g_dbgScState = 0;
volatile uint8_t g_dbgSpecialMode = 0;
volatile uint8_t g_dbgCalibAction = 0;   // 0无 1=180 2=90
volatile uint8_t g_dbgHoldDone = 0;
volatile uint8_t g_dbgCalibResult = 0;   // 0未尝试 1成功 2失败
volatile uint32_t g_dbgHoldMs = 0;

static BusServo *g_busServo = nullptr;
static bool g_busServoTorqueEnabled = false;
/* ---------- 总线舵机故障状态（0x41），用于板载 RGB 指示 ---------- */
volatile uint8_t g_busServoStatus[2] = {0u, 0u};
static uint32_t g_busServoStatusTick = 0;
static uint8_t g_busServoStatusIndex = 0;
/* ---------- 总线舵机状态调试（上电读一次，Watch 里看） ---------- */
volatile uint8_t g_dbgBusSt1 = 0, g_dbgBusSt2 = 0;      /* 0x41 状态/错误标志 */
volatile uint8_t g_dbgBusTmp1 = 0, g_dbgBusTmp2 = 0;    /* 0x3F 温度 */
volatile uint8_t g_dbgBusVolt1 = 0, g_dbgBusVolt2 = 0;  /* 0x3E 电压(0.1V) */
volatile int16_t g_dbgBusLoad1 = 0, g_dbgBusLoad2 = 0;  /* 0x3C 负载 */
volatile uint8_t g_dbgBusOk1 = 0, g_dbgBusOk2 = 0;      /* 读取是否成功 */
static uint32_t g_busServoLastTorqueOffTick = 0;
float g_busServoAngle = BUS_SERVO_ANGLE_INIT;

namespace {
constexpr uint8_t kBusServoCount = 2;
constexpr uint8_t kBusServoIds[kBusServoCount] = {
    BUS_SERVO_ID_1, BUS_SERVO_ID_2
};
constexpr uint16_t kBusServoSpeed = 600;
constexpr uint8_t kBusServoAcceleration = 30;
constexpr uint32_t kBusServoCommandRefreshMs = 50;
constexpr uint32_t kBusServoTorqueOffRetryMs = 100;
constexpr uint32_t kBusServoPingIntervalMs = 100;
constexpr uint32_t kBusServoAckTimeoutMs = 1000;
constexpr float kBusServoAngleStickDeadzone = 0.05f;
constexpr float kBusServoAngleDtMax = 0.05f;
constexpr float kBusServoRawPerDegree = 4096.0f / 360.0f;
constexpr float kBusServoCalibStickThreshold = 0.5f;
constexpr uint32_t kBusServoCalibHoldMs = 2000u;
constexpr uint32_t kBusServoSpecialModeBlinkMs = 300u;
constexpr uint32_t kBusServoFeedbackFlashMs = 1500u;
constexpr uint32_t kBusServoFeedbackBlinkMs = 100u;
constexpr uint32_t kBusServoBeepMs = 120u;
constexpr uint32_t kBusServoBeepReadyMs = 80u;
constexpr uint32_t kBusServoBeepGestureMs = 60u;
constexpr float kBusServoBeepReadyHz = 1000.0f;
constexpr float kBusServoBeepGestureHz = 1500.0f;
constexpr float kBusServoBeepSuccessHz = 2000.0f;
constexpr float kBusServoBeepFailHz = 800.0f;
constexpr float kBusServoBeepVolume = 1.0f;

enum BusServoCalibAction : uint8_t {
    BUS_CALIB_NONE = 0,
    BUS_CALIB_180,
    BUS_CALIB_0
};

enum StickDirection : uint8_t {
    STICK_CENTER = 0,
    STICK_UP,
    STICK_UP_RIGHT,
    STICK_RIGHT,
    STICK_DOWN_RIGHT,
    STICK_DOWN,
    STICK_DOWN_LEFT,
    STICK_LEFT,
    STICK_UP_LEFT
};

struct BusServoSettingGesture {
    StickDirection left;
    StickDirection right;
    BusServoCalibAction action;
};

// 新增特殊设置方向时，在这里加一行即可
const BusServoSettingGesture kBusServoSettingGestures[] = {
    {STICK_UP_LEFT,    STICK_UP_RIGHT,    BUS_CALIB_180},
    {STICK_DOWN_LEFT,  STICK_DOWN_RIGHT,  BUS_CALIB_0},
};

uint32_t g_busServoLastAckTick[kBusServoCount] = {0, 0};
bool g_busServoAcked[kBusServoCount] = {false, false};
uint8_t g_busServoPingIndex = 0;
uint32_t g_busServoLastPingTick = 0;

int16_t g_busServoCalibOffset[kBusServoCount] = {0, 0};
BusServoCalibAction g_busServoCalibHoldAction = BUS_CALIB_NONE;
uint32_t g_busServoCalibHoldStartTick = 0;
bool g_busServoCalibHoldDone = false;
uint32_t g_busServoFeedbackStartTick = 0;
bool g_busServoFeedbackSuccess = false;
bool g_busServoBeepActive = false;
uint32_t g_busServoBeepOffTick = 0;
bool g_busServoSpecialModeActive = false;

void BusServoBeep(float freq, uint32_t durationMs) {
    g_busServoBeepOffTick = HAL_GetTick() + durationMs;
    g_busServoBeepActive = true;
    bsp_BuzzerOn(freq, kBusServoBeepVolume);
}

void StartBusServoCalibrationFeedback(bool success) {
    g_busServoFeedbackStartTick = HAL_GetTick();
    g_busServoFeedbackSuccess = success;
    BusServoBeep(success ? kBusServoBeepSuccessHz : kBusServoBeepFailHz,
                 kBusServoBeepMs);
}

void BusServoCalibrationFeedbackHandle() {
    if (g_busServoBeepActive &&
        ((int32_t)(HAL_GetTick() - g_busServoBeepOffTick) >= 0)) {
        bsp_BuzzerOff();
        g_busServoBeepActive = false;
    }
}

/**
 * @brief 读两颗总线舵机的状态/温度/电压/负载（上电调用一次）
 * @note  0x41=状态, 0x3F=温度, 0x3E=电压(0.1V), 0x3C=负载
 */
static void BusServoDumpStatus() {
    if (g_busServo == nullptr) {
        return;
    }

    for (uint8_t i = 0; i < kBusServoCount; ++i) {
        uint8_t d[2] = {0u, 0u};
        const uint8_t id = kBusServoIds[i];
        const bool ok = g_busServo->readRegs(id, 0x41u, 1u, d);

        if (i == 0u) {
            g_dbgBusOk1 = ok ? 1u : 0u;
            g_dbgBusSt1 = ok ? d[0] : 0u;
        } else {
            g_dbgBusOk2 = ok ? 1u : 0u;
            g_dbgBusSt2 = ok ? d[0] : 0u;
        }

        uint8_t t = 0u;
        if (g_busServo->readRegs(id, 0x3Fu, 1u, &t)) {
            if (i == 0u) { g_dbgBusTmp1 = t; } else { g_dbgBusTmp2 = t; }
        }

        uint8_t v = 0u;
        if (g_busServo->readRegs(id, 0x3Eu, 1u, &v)) {
            if (i == 0u) { g_dbgBusVolt1 = v; } else { g_dbgBusVolt2 = v; }
        }

        uint8_t ld[2] = {0u, 0u};
        if (g_busServo->readRegs(id, 0x3Cu, 2u, ld)) {
            const int16_t load = (int16_t)((uint16_t)ld[0] | ((uint16_t)ld[1] << 8));
            if (i == 0u) { g_dbgBusLoad1 = load; } else { g_dbgBusLoad2 = load; }
        }
    }
}
/**
 * @brief 轮流读取两颗舵机的 0x41 故障状态（每 300 ms 一次）
 */
static void BusServoPollStatus() {
    if (g_busServo == nullptr) {
        return;
    }
    const uint32_t now = HAL_GetTick();
    if ((now - g_busServoStatusTick) < 300u) {
        return;
    }
    g_busServoStatusTick = now;

    uint8_t d[1] = {0u};
    const uint8_t id = kBusServoIds[g_busServoStatusIndex];
    if (g_busServo->readRegs(id, 0x41u, 1u, d)) {
        g_busServoStatus[g_busServoStatusIndex] = d[0];
    } else {
        g_busServoStatus[g_busServoStatusIndex] = 0u;
    }
    g_busServoStatusIndex = (uint8_t)(g_busServoStatusIndex ^ 1u);
}

bool IsBusServoSpecialSettingMode() {
    if (!RemoteControl::isOnline()) {
        return false;
    }
    /* v2.0: SA up + SB up + SC down + SD down = special setting mode */
    return (RemoteControl::rcInfo.SA == UP_POS) &&
           (RemoteControl::rcInfo.SB == UP_POS) &&
           (RemoteControl::rcInfo.SC == DOWN_POS) &&
           (RemoteControl::rcInfo.SD == DOWN_POS);
}

StickDirection GetStickDirection(float col, float rol, float threshold) {
    const bool up = col > threshold;
    const bool down = col < -threshold;
    const bool left = rol < -threshold;
    const bool right = rol > threshold;

    if (up && left)    return STICK_UP_LEFT;
    if (up && right)   return STICK_UP_RIGHT;
    if (down && left)  return STICK_DOWN_LEFT;
    if (down && right) return STICK_DOWN_RIGHT;
    if (up)            return STICK_UP;
    if (down)          return STICK_DOWN;
    if (left)          return STICK_LEFT;
    if (right)         return STICK_RIGHT;
    return STICK_CENTER;
}

int16_t WrapBusServoRaw(int32_t raw) {
    raw %= 4096;
    if (raw < 0) {
        raw += 4096;
    }
    return (int16_t)raw;
}

int16_t BusServoBasePos(uint8_t index, float angleDeg) {
    const float rawf = angleDeg * kBusServoRawPerDegree;
    int32_t raw = (int32_t)(rawf + ((rawf >= 0.0f) ? 0.5f : -0.5f));
    raw = WrapBusServoRaw(raw);
#if BUS_SERVO_MIRROR_SECOND
    if (index == 1u) {
        raw = 4095 - raw;
    }
#endif
    return (int16_t)raw;
}

int16_t BusServoCorrectedPos(uint8_t index, int16_t basePos) {
    return WrapBusServoRaw((int32_t)basePos + g_busServoCalibOffset[index]);
}

bool BusServoWriteTorque(bool enable) {
    if (g_busServo == nullptr) {
        return false;
    }
    return g_busServo->syncWriteTorque(kBusServoIds, enable, kBusServoCount);
}

void LoadBusServoCalibration() {
    if (flashData.busServoCalibMagic != BUS_SERVO_CALIB_MAGIC) {
        return;
    }
    if ((flashData.busServoCalibOffset[0] < -2048) ||
        (flashData.busServoCalibOffset[0] > 2047) ||
        (flashData.busServoCalibOffset[1] < -2048) ||
        (flashData.busServoCalibOffset[1] > 2047)) {
        return;
    }

    for (uint8_t i = 0; i < kBusServoCount; ++i) {
        g_busServoCalibOffset[i] = flashData.busServoCalibOffset[i];
    }
}

bool SaveBusServoCalibration(const int16_t *offsets, float referenceDeg) {
    const int16_t prevOffsets[kBusServoCount] = {
        flashData.busServoCalibOffset[0], flashData.busServoCalibOffset[1]
    };
    const int16_t prevReferenceDeg = flashData.busServoCalibReferenceDeg;
    const uint16_t prevReserved = flashData.busServoCalibReserved;
    const uint32_t prevMagic = flashData.busServoCalibMagic;

    for (uint8_t i = 0; i < kBusServoCount; ++i) {
        flashData.busServoCalibOffset[i] = offsets[i];
    }
    flashData.busServoCalibReferenceDeg = (int16_t)referenceDeg;
    flashData.busServoCalibReserved = 0;
    flashData.busServoCalibMagic = BUS_SERVO_CALIB_MAGIC;

    if (!bsp_flash_write(&flashData)) {
        for (uint8_t i = 0; i < kBusServoCount; ++i) {
            flashData.busServoCalibOffset[i] = prevOffsets[i];
        }
        flashData.busServoCalibReferenceDeg = prevReferenceDeg;
        flashData.busServoCalibReserved = prevReserved;
        flashData.busServoCalibMagic = prevMagic;
        return false;
    }
    return true;
}

bool ApplyBusServoCalibration(BusServoCalibAction action) {
    if (g_busServo == nullptr) {
        return false;
    }

    int16_t current[kBusServoCount];
    for (uint8_t i = 0; i < kBusServoCount; ++i) {
        if (!g_busServo->readPos(kBusServoIds[i], current[i])) {
            return false;
        }
    }

    const float referenceDeg = (action == BUS_CALIB_0) ? 0.0f : 180.0f;
    int16_t offsets[kBusServoCount];
    for (uint8_t i = 0; i < kBusServoCount; ++i) {
        int32_t offset = (int32_t)current[i] - BusServoBasePos(i, referenceDeg);
        offset %= 4096;
        if (offset > 2047) {
            offset -= 4096;
        }
        if (offset < -2048) {
            offset += 4096;
        }
        offsets[i] = (int16_t)offset;
    }

    if (!SaveBusServoCalibration(offsets, referenceDeg)) {
        return false;
    }
    for (uint8_t i = 0; i < kBusServoCount; ++i) {
        g_busServoCalibOffset[i] = offsets[i];
    }
    return true;
}

BusServoCalibAction DetectBusServoCalibAction() {
    if (!IsBusServoSpecialSettingMode()) {
        return BUS_CALIB_NONE;
    }

    const RC_Info_t &rc = RemoteControl::rcInfo;
    const float t = kBusServoCalibStickThreshold;
    const StickDirection left =
        GetStickDirection(rc.left_col, rc.left_rol, t);
    const StickDirection right =
        GetStickDirection(rc.right_col, rc.right_rol, t);

    for (const BusServoSettingGesture &gesture : kBusServoSettingGestures) {
        if ((gesture.left == left) && (gesture.right == right)) {
            return gesture.action;
        }
    }
    return BUS_CALIB_NONE;
}

uint8_t SwitchStateCode(SWITCH_STATE_E state) {
    if (state == UP_POS) {
        return 0;
    }
    if (state == DOWN_POS) {
        return 2;
    }
    return 1;
}

void UpdateBusServoWatch() {
    const RC_Info_t &rc = RemoteControl::rcInfo;
    const float t = kBusServoCalibStickThreshold;

    g_dbgLeftCol = rc.left_col;
    g_dbgLeftRol = rc.left_rol;
    g_dbgRightCol = rc.right_col;
    g_dbgRightRol = rc.right_rol;
    g_dbgLeftDir = (uint8_t)GetStickDirection(rc.left_col, rc.left_rol, t);
    g_dbgRightDir = (uint8_t)GetStickDirection(rc.right_col, rc.right_rol, t);

    g_dbgSaRaw = (uint16_t)rc.optional[0];
    g_dbgSdRaw = (uint16_t)rc.optional[1];
    g_dbgSbRaw = (uint16_t)RemoteControl::rc_ctrl.rc.s[0];
    g_dbgScRaw = (uint16_t)RemoteControl::rc_ctrl.rc.s[1];
    g_dbgSaState = SwitchStateCode(rc.SA);
    g_dbgSdState = SwitchStateCode(rc.SD);
    g_dbgSbState = SwitchStateCode(rc.SB);
    g_dbgScState = SwitchStateCode(rc.SC);

    g_dbgSpecialMode = IsBusServoSpecialSettingMode() ? 1u : 0u;
    g_dbgCalibAction = (uint8_t)DetectBusServoCalibAction();
    g_dbgHoldDone = g_busServoCalibHoldDone ? 1u : 0u;
    g_dbgHoldMs = (g_busServoCalibHoldAction != BUS_CALIB_NONE)
                      ? (HAL_GetTick() - g_busServoCalibHoldStartTick)
                      : 0u;
}

void BusServoCalibrationHandle() {
    const bool specialMode = IsBusServoSpecialSettingMode();
    if (specialMode && !g_busServoSpecialModeActive) {
        BusServoBeep(kBusServoBeepReadyHz, kBusServoBeepReadyMs);
    }
    g_busServoSpecialModeActive = specialMode;

    const BusServoCalibAction action = DetectBusServoCalibAction();
    if (action == BUS_CALIB_NONE) {
        g_busServoCalibHoldAction = BUS_CALIB_NONE;
        g_busServoCalibHoldDone = false;
        return;
    }
    if (g_busServoCalibHoldDone) {
        return;
    }

    const uint32_t now = HAL_GetTick();
    if (action != g_busServoCalibHoldAction) {
        BusServoBeep(kBusServoBeepGestureHz, kBusServoBeepGestureMs);
        g_busServoCalibHoldAction = action;
        g_busServoCalibHoldStartTick = now;
        return;
    }
    if ((now - g_busServoCalibHoldStartTick) < kBusServoCalibHoldMs) {
        return;
    }

    const bool ok = ApplyBusServoCalibration(action);
    g_busServoCalibHoldDone = true;
    g_dbgCalibResult = ok ? 1u : 2u;
    StartBusServoCalibrationFeedback(ok);
}
}

bool BusServoCalibrationFeedbackColor(uint32_t *color) {
    if (color == nullptr) {
        return false;
    }

    if (g_busServoFeedbackStartTick != 0u) {
        const uint32_t elapsed = HAL_GetTick() - g_busServoFeedbackStartTick;
        if (elapsed < kBusServoFeedbackFlashMs) {
            if (((elapsed / kBusServoFeedbackBlinkMs) & 1u) == 0u) {
                *color = g_busServoFeedbackSuccess ? 0xFF00FFFFu : 0xFFFF0000u;
            } else {
                *color = 0u;
            }
            return true;
        }
        g_busServoFeedbackStartTick = 0;
    }

    if (!IsBusServoSpecialSettingMode()) {
        return false;
    }

    // 特殊设置模式已进入：蓝灯慢闪，提示可以开始推摇杆
    const uint32_t phase =
        HAL_GetTick() % (2u * kBusServoSpecialModeBlinkMs);
    if (phase < kBusServoSpecialModeBlinkMs) {
        *color = 0x800000FFu;
    } else {
        *color = 0u;
    }
    return true;
}

BUS_SERVO_LINK_STATE_E BusServoLinkState() {
    const uint32_t now = HAL_GetTick();
    uint8_t onlineCount = 0;
    for (uint8_t i = 0; i < kBusServoCount; ++i) {
        if (g_busServoAcked[i] &&
            ((now - g_busServoLastAckTick[i]) <= kBusServoAckTimeoutMs)) {
            onlineCount++;
        }
    }

    if (onlineCount == 0) {
        return BUS_SERVO_LINK_NONE;
    }
    if (onlineCount < kBusServoCount) {
        return BUS_SERVO_LINK_PARTIAL;
    }
    return BUS_SERVO_LINK_ALL;
}

// ================= plate (rack) servo: open loop by time, auto-follow bus angle =================
static Servo *g_plateServo = nullptr;
static uint32_t g_plateStopTick = 0;

enum PlateState {
    PLATE_IDLE = 0,
    PLATE_MOVING
};

static PlateState g_plateState = PLATE_IDLE;
static int32_t g_platePosMs = 0;
static int32_t g_plateTargetMs = 0;
static uint32_t g_plateMoveMs = 0;
static int8_t g_plateMoveDir = 0;

/* 自动跟随 / 手动覆盖 状态 */
static bool g_plateManual = false;
static float g_plateManualStartAngle = 0.0f;
static int8_t g_plateAutoTarget = -1;      /* -1 未知, 0 收到底, 1 伸到底 */

static void PlateDrive(float speed) {
    if (g_plateServo != nullptr) {
        g_plateServo->SetTargetSpeed(speed);
    }
}

static void PlateBeginMove() {
    if (g_plateServo == nullptr) {
        return;
    }
    if (g_plateTargetMs == g_platePosMs) {
        g_plateState = PLATE_IDLE;
        return;
    }
    if (g_plateTargetMs > g_platePosMs) {
        g_plateMoveDir = PLATE_DIR_EXTEND;
        g_plateMoveMs  = (uint32_t)(g_plateTargetMs - g_platePosMs);
    } else {
        g_plateMoveDir = PLATE_DIR_RETRACT;
        g_plateMoveMs  = (uint32_t)(g_platePosMs - g_plateTargetMs);
    }
    PlateDrive((float)g_plateMoveDir * PLATE_SPEED);
    g_plateStopTick = HAL_GetTick() + g_plateMoveMs;
    g_plateState = PLATE_MOVING;
}

static void PlateGoto(int32_t targetMs) {
    if (g_plateServo == nullptr) {
        return;
    }
    if (targetMs > (int32_t)PLATE_POS_HIGH_MS) {
        targetMs = (int32_t)PLATE_POS_HIGH_MS;
    }
    if (targetMs < 0) {
        targetMs = 0;
    }
    g_plateTargetMs = targetMs;
    PlateBeginMove();
}

static void PlateStop() {
    if ((g_plateState == PLATE_MOVING) && (g_plateStopTick != 0u)) {
        const uint32_t now = HAL_GetTick();
        if (now < g_plateStopTick) {
            const uint32_t remain = g_plateStopTick - now;
            const int32_t done = (int32_t)(g_plateMoveMs - remain);
            if (g_plateMoveDir == PLATE_DIR_EXTEND) {
                g_platePosMs += done;
            } else {
                g_platePosMs -= done;
            }
            if (g_platePosMs > (int32_t)PLATE_POS_HIGH_MS) {
                g_platePosMs = (int32_t)PLATE_POS_HIGH_MS;
            }
            if (g_platePosMs < 0) {
                g_platePosMs = 0;
            }
        }
    }

    g_plateStopTick = 0u;
    g_plateState = PLATE_IDLE;
    g_plateMoveDir = 0;
    PlateDrive(0.0f);
    if (g_plateServo != nullptr) {
        g_plateServo->Handle();
    }
}

static void PlateHandle() {
    const uint32_t now = HAL_GetTick();
    if ((g_plateStopTick != 0u) && (now >= g_plateStopTick)) {
        g_plateStopTick = 0u;
        PlateDrive(0.0f);
        if (g_plateState == PLATE_MOVING) {
            g_platePosMs = g_plateTargetMs;
            g_plateState = PLATE_IDLE;
        }
    }
    if (g_plateServo != nullptr) {
        g_plateServo->Handle();
    }
}

/**
 * @brief 齿条控制：默认随总线舵机角度自动伸缩，摇杆可手动覆盖
 * @param manualStick 手动摇杆（左摇杆左右，右正左负）
 * @param busAngle    当前总线舵机角度（度）
 *
 * 规则：
 *   角度 > 80+2 度  -> 伸到底
 *   角度 < 80-2 度  -> 收到底
 *   手动推摇杆      -> 接管，自动暂停，松手后保持位置
 *   手动后角度跨过 80 度 -> 自动重新接管
 */
void PlateControl(float manualStick, float busAngle) {
    static uint32_t lastTick = 0;
    const uint32_t now = HAL_GetTick();
    uint32_t dt = 0;
    if (lastTick != 0u) {
        dt = now - lastTick;
    }
    lastTick = now;
    if (dt > 50u) {
        dt = 50u;
    }

    const bool manualNow = (manualStick > PLATE_MANUAL_DEADZONE) ||
                           (manualStick < -PLATE_MANUAL_DEADZONE);

    if (manualNow) {
        if (!g_plateManual) {
            g_plateManual = true;
            g_plateManualStartAngle = busAngle;
            g_plateAutoTarget = -1;
        }

        /* 手动优先：取消自动动作，直接给速度 */
        g_plateStopTick = 0u;
        g_plateState = PLATE_IDLE;

        const float dir = (manualStick > 0.0f) ? PLATE_MANUAL_DIR_SIGN : -PLATE_MANUAL_DIR_SIGN;
        PlateDrive(dir * PLATE_MANUAL_SPEED);

        /* 按时间估算位置：手动速度与正常速度不同，需要按比例换算 */
        const float ratio = PLATE_MANUAL_SPEED / PLATE_SPEED;
        const int32_t delta = (int32_t)((float)dt * ratio + 0.5f);
        if (dir > 0.0f) {
            g_platePosMs += delta;
        } else {
            g_platePosMs -= delta;
        }
        if (g_platePosMs > (int32_t)PLATE_POS_HIGH_MS) {
            g_platePosMs = (int32_t)PLATE_POS_HIGH_MS;
        }
        if (g_platePosMs < 0) {
            g_platePosMs = 0;
        }
        g_plateTargetMs = g_platePosMs;
        return;
    }

    if (g_plateManual) {
        /* 松手：停住并保持位置 */
        PlateDrive(0.0f);

        /* 角度跨过阈值 -> 恢复自动 */
        const float before = g_plateManualStartAngle - PLATE_AUTO_THRESHOLD_DEG;
        const float after  = busAngle - PLATE_AUTO_THRESHOLD_DEG;
        if ((before > 0.0f) == (after > 0.0f)) {
            return;         /* 还没跨过，保持手动锁存 */
        }
        g_plateManual = false;
        g_plateAutoTarget = -1;
    }

    /* 自动模式：带死区，防止在阈值附近抖动 */
    int8_t want;
    if (busAngle > (PLATE_AUTO_THRESHOLD_DEG + PLATE_AUTO_HYST_DEG)) {
        want = 1;
    } else if (busAngle < (PLATE_AUTO_THRESHOLD_DEG - PLATE_AUTO_HYST_DEG)) {
        want = 0;
    } else {
        return;             /* 死区内保持当前位置 */
    }

    if (want != g_plateAutoTarget) {
        g_plateAutoTarget = want;
        PlateGoto(want ? (int32_t)PLATE_POS_HIGH_MS : 0);
    }
}
// ================= end plate =================


/**
 * @brief 故障状态 -> 板载 RGB 颜色（闪动次数与舵机自身 LED 一致）
 * @note  bit0 电压(闪1) / bit1 磁编码(闪2) / bit5 负载(闪3) / bit2 温度(闪4) / bit3 电流(闪5)
 */
bool BusServoFaultColor(uint32_t *color) {
    if (color == nullptr) {
        return false;
    }

    const uint8_t status = (uint8_t)(g_busServoStatus[0] | g_busServoStatus[1]);
    if (status == 0u) {
        return false;
    }

    uint32_t rgb = 0u;
    uint8_t flashes = 1u;

    if ((status & 0x01u) != 0u) {          /* 电压 */
        rgb = 0xFF8000FFu; flashes = 1u;
    } else if ((status & 0x02u) != 0u) {   /* 磁编码 */
        rgb = 0xFF0000FFu; flashes = 2u;
    } else if ((status & 0x20u) != 0u) {   /* 负载 */
        rgb = 0xFFFFFF00u; flashes = 3u;
    } else if ((status & 0x04u) != 0u) {   /* 温度 */
        rgb = 0xFFFF0000u; flashes = 4u;
    } else {                               /* 电流 */
        rgb = 0xFFFF8000u; flashes = 5u;
    }

    const uint32_t periodMs = 1000u;
    const uint32_t slot = periodMs / flashes;
    const uint32_t phase = HAL_GetTick() % periodMs;
    const uint32_t idx = phase / slot;
    const uint32_t onTime = (slot >= 60u) ? 60u : (slot / 2u);
    *color = ((phase - idx * slot) < onTime) ? rgb : 0u;
    return true;
}

PID_Regulator_t userPidRegulator = {
        .kp = 60,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 10000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 10000 //2006电机输出电流上限，可以调小，勿调大
};

MOTOR_INIT_t userMotorInit = {
        .speedPIDp = &userPidRegulator,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 36.0f,
        .ctrlType = SPEED_Single,
};

// 在这里初始化除了4个M3508轮电机之外的电机
Motor UserMotor(MOTOR_ID_5,&userMotorInit);

SERVO_INIT_T TurnL_Servo_Init = {
        .servoType = SPEED_360,
        .servoID = SERVO_ID_1,
        .firstAngle = 0,
        .angleLimit_Min = 0,
        .angleLimit_Max = 180
};
Servo TurnLServo(&TurnL_Servo_Init);

SERVO_INIT_T TurnR_Servo_Init = {
        .servoType = SPEED_360,
        .servoID = SERVO_ID_2,
        .firstAngle = 0,
        .angleLimit_Min = 0,
        .angleLimit_Max = 180
};
Servo TurnRServo(&TurnR_Servo_Init);

SERVO_INIT_T ShovelL_Servo_Init = {
        .servoType = SPEED_360,
        .servoID = SERVO_ID_3,
        .firstAngle = 0,
        .angleLimit_Min = 0,
        .angleLimit_Max = 180
};
Servo ShovelLServo(&ShovelL_Servo_Init);

SERVO_INIT_T ShovelR_Servo_Init = {
        .servoType = SPEED_360,
        .servoID = SERVO_ID_4,
        .firstAngle = 0,
        .angleLimit_Min = 0,
        .angleLimit_Max = 180
};
Servo ShovelRServo(&ShovelR_Servo_Init);

/***
 * 在这里放入xxx.stop()即可使舵机,电机在遥控器急停挡位断电
 */
void UserStop(){
    UserMotor.Stop();

    pushrod_all_off();

    if (g_busServo != nullptr) {
        const uint32_t now = HAL_GetTick();
        if (g_busServoTorqueEnabled ||
            ((now - g_busServoLastTorqueOffTick) >= kBusServoTorqueOffRetryMs)) {
            BusServoWriteTorque(false);
            g_busServoTorqueEnabled = false;
            g_busServoLastTorqueOffTick = now;
        }
    }
    TurnLServo.stop();
    TurnRServo.stop();

    ShovelLServo.stop();
    ShovelRServo.stop();

    PlateStop();
}

/***
 * 在这里写入初始化内容
 */
void UserInit(){
    // plate servo on PWM5 (PC6 / TIM8_CH1): no homing, assume start at low position
    static SERVO_INIT_T plateInit = {SPEED_360, PLATE_SERVO_ID, 0, 0, 180};
    static Servo plate(&plateInit);
    g_plateServo = &plate;
    g_plateServo->SetTargetSpeed(0.0f);
    g_plateServo->Handle();
    g_platePosMs = PLATE_POS_LOW_MS;
    g_plateTargetMs = PLATE_POS_LOW_MS;
    g_plateState = PLATE_IDLE;
    TurnLServo.SetTargetSpeed(0.0f);
    TurnRServo.SetTargetSpeed(0.0f);
    ShovelLServo.SetTargetSpeed(0.0f);
    ShovelRServo.SetTargetSpeed(0.0f);
    TurnLServo.Handle();
    TurnRServo.Handle();
    ShovelLServo.Handle();
    ShovelRServo.Handle();

    // UART 已初始化，构造总线舵机对象
    static BusServo servo(BUS_SERVO_UART);
    g_busServo = &servo;
    // Re-apply the bus configuration after the object is constructed.
    servo.setBaud(BUS_SERVO_BAUD);
    LoadBusServoCalibration();
    g_busServoLastTorqueOffTick = HAL_GetTick() - kBusServoTorqueOffRetryMs;

    BusServoDumpStatus();   /* 上电读一次状态，Watch 里看 */
    g_busServoLastPingTick = HAL_GetTick() - kBusServoPingIntervalMs;
}

/**
 * @brief 按角度设置两个总线舵机（自动 clamp 并应用特殊设置修正）
 */
void BusServoSetAngle(float angle) {
    if (g_busServo == nullptr) {
        return;
    }

    if (angle < BUS_SERVO_ANGLE_MIN) {
        angle = BUS_SERVO_ANGLE_MIN;
    } else if (angle > BUS_SERVO_ANGLE_MAX) {
        angle = BUS_SERVO_ANGLE_MAX;
    }

    int16_t pos[kBusServoCount];
    for (uint8_t i = 0; i < kBusServoCount; ++i) {
        pos[i] = BusServoCorrectedPos(i, BusServoBasePos(i, angle));
    }

    if (BusServoWriteTorque(true)) {
        g_busServoTorqueEnabled = true;
    }
    g_busServo->syncWritePos(kBusServoIds, pos, kBusServoSpeed,
                             kBusServoAcceleration, kBusServoCount);
}

/**
 * @brief 总线舵机角度控制
 * @param stickUpDown 左摇杆上下，向上为正
 * @param enable true 时积分角度；false 时保持当前角度
 * @note  角度变化率 = 摇杆值 * BUS_SERVO_ANGLE_RATE_MAX
 */
void BusServoAngleControl(float stickUpDown, bool enable) {
    static uint32_t lastTick = 0;
    static uint32_t lastCommandTick = 0;
    const uint32_t now = HAL_GetTick();

    if (lastTick == 0u) {
        lastTick = now;
        lastCommandTick = now;
        return;
    }

    float dt = (now - lastTick) * 0.001f;
    lastTick = now;
    if (dt > kBusServoAngleDtMax) {
        dt = kBusServoAngleDtMax;
    }

    if (enable) {
        float stick = stickUpDown;
        if ((stick > -kBusServoAngleStickDeadzone) &&
            (stick < kBusServoAngleStickDeadzone)) {
            stick = 0.0f;
        }

        g_busServoAngle += stick * BUS_SERVO_ANGLE_RATE_MAX * dt;
        if (g_busServoAngle < BUS_SERVO_ANGLE_MIN) {
            g_busServoAngle = BUS_SERVO_ANGLE_MIN;
        } else if (g_busServoAngle > BUS_SERVO_ANGLE_MAX) {
            g_busServoAngle = BUS_SERVO_ANGLE_MAX;
        }
    }

    if ((now - lastCommandTick) >= kBusServoCommandRefreshMs) {
        BusServoSetAngle(g_busServoAngle);
        lastCommandTick = now;
    }
}

/***
 * 用户自定义任务主循环
 */
void UserHandle(){
    UserMotor.Handle();
    PlateHandle();          // plate servo state machine
    TurnLServo.Handle();
    TurnRServo.Handle();
    ShovelLServo.Handle();
    ShovelRServo.Handle();

    BusServoCalibrationFeedbackHandle();
    BusServoPollStatus();
    BusServoCalibrationHandle();
    UpdateBusServoWatch();

    // 轮流 ping 两个总线舵机，以最后成功回包时间判断在线状态
    if (g_busServo != nullptr) {
        const uint32_t now = HAL_GetTick();
        if ((now - g_busServoLastPingTick) >= kBusServoPingIntervalMs) {
            if (g_busServo->ping(kBusServoIds[g_busServoPingIndex])) {
                g_busServoAcked[g_busServoPingIndex] = true;
                g_busServoLastAckTick[g_busServoPingIndex] = now;
            }
            g_busServoPingIndex =
                (uint8_t)((g_busServoPingIndex + 1u) % kBusServoCount);
            g_busServoLastPingTick = now;
        }
    }
}
