//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_CHASSISTASK_H
#define RM_FRAME_C_CHASSISTASK_H
#include "main.h"
#include "Motor.h"

#define RPM2RADpS(num)(num * (3.1415926f/30.0f))
#define RADpS2RPM(num)(num / (3.1415926f/30.0f))
#define WHEEL_DIAMETER             0.1525f//麦轮直径 m
#define REDUCTION_RATIO 19.0f
#define TRACK_WIDTH            0.40f //轮距
#define WHEEL_BASE             0.38f //轴距

//底盘速度上限，遥控器满杆时对应的目标速度（手感主要靠这两个值调）
#define CHASSIS_MAX_LINEAR     3.0f  //平移速度上限 m/s
#define CHASSIS_MAX_ROT_RPM    40.0f //旋转速度上限 rpm

/*
 * 单个车轮的安装方向系数。
 * 电机正转（C620 收到正电流）让车前进时取 +1，反转才能前进时取 -1。
 * 四轮是镜像安装的，方向不一定一致；上车前用 UserTask.cpp 里的
 * WHEEL_SELFTEST 逐个确认，然后只改这里，不要改运动学公式。
 */
#define WHEEL_DIR_FL           1.0f
#define WHEEL_DIR_FR           1.0f
#define WHEEL_DIR_BL           1.0f
#define WHEEL_DIR_BR           1.0f

/*
 * 单轮点动自检（调试用，默认关闭）。
 * 打开后 CtrlHandle() 会忽略遥控器，只让 SELFTEST_WHEEL 号轮子以 SELFTEST_RPM 转，
 * 用来确认「ID ↔ 物理位置 ↔ 转向」。确认完请注释掉，否则车不会听遥控器。
 */
//#define WHEEL_SELFTEST
#ifdef WHEEL_SELFTEST
#define SELFTEST_WHEEL         1     //1=左前 2=右前 3=左后 4=右后
#define SELFTEST_RPM           100.0f
#endif

/*枚举类型定义------------------------------------------------------------*/

/*
 * 底盘任务错误信息枚举
 */
typedef enum{
    NONE
}CHASSIS_ERROR_E;

/*结构体定义--------------------------------------------------------------*/

/*
 * 底盘任务状态结构体
 */
typedef struct{

}CHASSIS_STATE_T;

/*
 * 底盘任务控制结构体
 */
typedef struct{


}CHASSIS_CONTROL_T;



/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/

extern CHASSIS_STATE_T     chassisState;
extern CHASSIS_CONTROL_T   chassisControl;

extern float FBVelocity,LRVelocity,RTVelocity;

extern Motor CMFL,CMFR,CMBL,CMBR;//四个底盘电机（调试/自检时可直接读状态）

/*外部函数声明-------------------------------------------------------------*/

void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity);
void ChassisStop();
void ChassisSetVelocity(float _fbV,float _lrV,float _rtV);
#ifdef WHEEL_SELFTEST
void ChassisSelfTest();
#endif
#endif //RM_FRAME_C_CHASSISTASK_H
