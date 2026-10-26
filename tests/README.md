# tests/

电脑端单元测试，**不参与烧录**，Keil 工程里没有包含它。

## 运行

在工程根目录（`RM2027_School_Comp_Frame/`）执行：

```bash
g++ -std=c++11 -O2 -Wall -IuserCode/tasks/Inc \
    -o tests/kinematics_test.exe \
    tests/kinematics_test.cpp userCode/tasks/Src/ChassisKinematics.cpp

./tests/kinematics_test.exe
```

退出码 0 = 全部通过，非 0 = 有失败项。

## 它验证什么

- 方向组合：前进全同号、右移对角反号、自转左右反号
- 单位换算：m/s → 输出轴 rpm
- 线性：速度翻倍输出翻倍；前进 + 横移可叠加
- 方向系数 `dirFl..dirBr` 只影响对应轮子
- 死区：小于阈值的摇杆输入归零
- 边界记录：满杆斜推 (3,3) 时两轮 751 rpm，超过 M3508 上限

## 它不验证什么

- CAN 通信、电调反馈、电机转向
- 遥控器解析、档位原始值
- PID 参数、机械安装

这些只能上板验证，见工程根目录的调试步骤（`RC_DEBUG` / `WHEEL_SELFTEST`）。
