#ifndef RM_FRAME_C_CHASSIS_KINEMATICS_HPP
#define RM_FRAME_C_CHASSIS_KINEMATICS_HPP

/**
 * @file    ChassisKinematics.hpp
 * @brief   四麦轮底盘解算：底盘三个方向的目标速度 <-> 四个轮子的目标转速
 *
 * 纯计算模块，不依赖 HAL、不依赖电机对象、不依赖遥控器。
 * 可以单独拿出来在电脑上算，也可以直接接到任意工程里。
 *
 * ============================ 坐标与符号约定 ============================
 *   vx > 0 ：前进
 *   vy > 0 ：向右平移
 *   wz > 0 ：顺时针旋转（车头向右转），与遥控器左摇杆向右推一致
 *
 *   FL = 左前  FR = 右前  BL = 左后  BR = 右后
 *
 * ================================ 单位 ================================
 *   输入 vx, vy ：m/s
 *   输入 wz     ：rpm（满杆旋转 = maxRotRpm）
 *   输出        ：减速箱输出轴 rpm —— 正好是 Motor::SetTargetSpeed() 需要的量
 *
 *   注意：这里输出的是「输出轴」转速，不是电机转子转速。
 *        框架里 state.speed = 反馈转速 / 减速比，所以两边是同一口径。
 *
 * =============================== 用法 ===============================
 * @code
 *   ChassisKinematics::Config cfg;
 *   cfg.dirFr = -1.0f;                        // 右前轮装反了，只改这一个系数
 *   ChassisKinematics kin(cfg);
 *
 *   WheelTargetRpm w = kin.solve(1.0f, 0.0f, 0.0f);   // 前进 1 m/s
 *   // 或者直接吃归一化摇杆：
 *   w = kin.fromStick(remote.right_col, remote.right_rol, remote.left_rol);
 *
 *   CMFL.SetTargetSpeed(w.fl);
 *   CMFR.SetTargetSpeed(w.fr);
 *   CMBL.SetTargetSpeed(w.bl);
 *   CMBR.SetTargetSpeed(w.br);
 * @endcode
 */
struct WheelTargetRpm {
    float fl;
    float fr;
    float bl;
    float br;
};

class ChassisKinematics {
public:
    /**
     * @brief 解算参数。所有可调量集中在这里，改参数不用动公式。
     */
    struct Config {
        float wheelDiameter = 0.1525f;   // 麦轮直径，m
        float trackWidth    = 0.40f;     // 轮距（左右轮中心距），m
        float wheelBase     = 0.38f;     // 轴距（前后轮中心距），m

        // 单个轮子的安装方向：电机正转(C620 正电流)使车前进取 +1，反转才前进取 -1
        float dirFl = 1.0f;
        float dirFr = 1.0f;
        float dirBl = 1.0f;
        float dirBr = 1.0f;

        // 下面三项只给 fromStick() 用，solve() 不碰
        float maxLinear = 3.0f;          // 满杆平移速度，m/s
        float maxRotRpm = 40.0f;         // 满杆旋转速度，rpm
        float deadzone  = 0.02f;         // 摇杆死区（归一化值）
    };

    ChassisKinematics();
    explicit ChassisKinematics(const Config &cfg);

    /**
     * @brief 第三层：底盘速度 -> 四个轮子目标转速
     * @param vx     前进速度，m/s
     * @param vy     右移速度，m/s
     * @param wzRpm  旋转速度，rpm，顺时针为正
     * @return 四个轮子的输出轴目标转速，rpm
     */
    WheelTargetRpm solve(float vx, float vy, float wzRpm) const;

    /**
     * @brief 第二层 + 第三层：归一化摇杆 -> 四个轮子目标转速
     * @param fwd     前后摇杆，-1~1，向前为正
     * @param strafe  左右摇杆，-1~1，向右为正
     * @param turn    旋转摇杆，-1~1，向右为正
     * @return 四个轮子的输出轴目标转速，rpm（已含死区与满杆限幅）
     */
    WheelTargetRpm fromStick(float fwd, float strafe, float turn) const;

    const Config &config() const { return cfg_; }

private:
    Config cfg_;
};

#endif // RM_FRAME_C_CHASSIS_KINEMATICS_HPP
