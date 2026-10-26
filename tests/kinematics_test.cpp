/**
 * @file    kinematics_test.cpp
 * @brief   ChassisKinematics 的电脑端单元测试（不参与烧录）
 *
 * 编译运行（在工程根目录）：
 *   g++ -std=c++11 -O2 -o tests/kinematics_test.exe tests/kinematics_test.cpp userCode/tasks/Src/ChassisKinematics.cpp
 *   ./tests/kinematics_test.exe
 *
 * 只验证“纯数学”这一层：方向组合、单位换算、死区、线性叠加。
 * 硬件行为（CAN、电机、遥控器）这里测不了，得上板。
 */
#include <cmath>
#include <cstdio>

#include "../userCode/tasks/Inc/ChassisKinematics.hpp"

static int g_failures = 0;

static void expectNear(const char *name, float got, float want, float tol = 0.5f) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("    %-6s got=%9.2f  want=%9.2f  %s\n", name, got, want, ok ? "OK" : "FAIL");
    if (!ok) {
        ++g_failures;
    }
}

int main() {
    ChassisKinematics kin;
    WheelTargetRpm w;

    std::printf("[1] 静止：所有输入为 0，四个轮子都该是 0\n");
    w = kin.solve(0.0f, 0.0f, 0.0f);
    expectNear("fl", w.fl, 0.0f);
    expectNear("fr", w.fr, 0.0f);
    expectNear("bl", w.bl, 0.0f);
    expectNear("br", w.br, 0.0f);

    std::printf("[2] 前进 1 m/s：四个轮子同向同速 125.2 rpm\n");
    w = kin.solve(1.0f, 0.0f, 0.0f);
    expectNear("fl", w.fl, 125.24f);
    expectNear("fr", w.fr, 125.24f);
    expectNear("bl", w.bl, 125.24f);
    expectNear("br", w.br, 125.24f);

    std::printf("[3] 右移 1 m/s：对角反号（FL/BR 倒转，FR/BL 正转）\n");
    w = kin.solve(0.0f, 1.0f, 0.0f);
    expectNear("fl", w.fl, -125.24f);
    expectNear("fr", w.fr, 125.24f);
    expectNear("bl", w.bl, 125.24f);
    expectNear("br", w.br, -125.24f);

    std::printf("[4] 顺时针 40 rpm：左侧向前、右侧向后（±204.6 rpm）\n");
    w = kin.solve(0.0f, 0.0f, 40.0f);
    expectNear("fl", w.fl, 204.59f);
    expectNear("fr", w.fr, -204.59f);
    expectNear("bl", w.bl, 204.59f);
    expectNear("br", w.br, -204.59f);

    std::printf("[5] 齐次性：速度翻倍，输出也翻倍\n");
    {
        const WheelTargetRpm a = kin.solve(1.0f, 0.0f, 0.0f);
        const WheelTargetRpm b = kin.solve(2.0f, 0.0f, 0.0f);
        expectNear("fl", b.fl, a.fl * 2.0f);
        expectNear("br", b.br, a.br * 2.0f);
    }

    std::printf("[6] 叠加性：前进 + 右移 == 两个单独结果相加\n");
    {
        const WheelTargetRpm x = kin.solve(1.0f, 1.0f, 0.0f);
        const WheelTargetRpm a = kin.solve(1.0f, 0.0f, 0.0f);
        const WheelTargetRpm b = kin.solve(0.0f, 1.0f, 0.0f);
        expectNear("fl", x.fl, a.fl + b.fl);
        expectNear("fr", x.fr, a.fr + b.fr);
        expectNear("bl", x.bl, a.bl + b.bl);
        expectNear("br", x.br, a.br + b.br);
    }

    std::printf("[7] 方向系数：dirFr = -1 只翻转右前轮\n");
    {
        ChassisKinematics::Config cfg;
        cfg.dirFr = -1.0f;
        ChassisKinematics flipped(cfg);
        w = flipped.solve(1.0f, 0.0f, 0.0f);
        expectNear("fl", w.fl, 125.24f);
        expectNear("fr", w.fr, -125.24f);
        expectNear("bl", w.bl, 125.24f);
        expectNear("br", w.br, 125.24f);
    }

    std::printf("[8] 摇杆满杆前进：3 m/s -> 375.7 rpm\n");
    w = kin.fromStick(1.0f, 0.0f, 0.0f);
    expectNear("fl", w.fl, 375.71f);
    expectNear("fr", w.fr, 375.71f);
    expectNear("bl", w.bl, 375.71f);
    expectNear("br", w.br, 375.71f);

    std::printf("[9] 死区：摇杆 0.01（< 0.02）应被当成 0\n");
    w = kin.fromStick(0.01f, -0.01f, 0.01f);
    expectNear("fl", w.fl, 0.0f);
    expectNear("fr", w.fr, 0.0f);
    expectNear("bl", w.bl, 0.0f);
    expectNear("br", w.br, 0.0f);

    std::printf("[10] 满杆斜推 (3,3)：FL/BR 应为 0，FR/BL 为 751.4（已超出 3508 上限）\n");
    w = kin.solve(3.0f, 3.0f, 0.0f);
    expectNear("fl", w.fl, 0.0f);
    expectNear("fr", w.fr, 751.42f);
    expectNear("bl", w.bl, 751.42f);
    expectNear("br", w.br, 0.0f);

    if (g_failures == 0) {
        std::printf("\n全部通过。\n");
        return 0;
    }
    std::printf("\n%d 项失败。\n", g_failures);
    return 1;
}
