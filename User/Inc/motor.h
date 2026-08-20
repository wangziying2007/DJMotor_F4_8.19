#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "pid.h"
#include "main.h"
#include <stdbool.h> /* 提供 bool / true / false */

// 辅助宏：绝对值、符号函数（用于编码器绕圈补偿）
#define ABS(x) (((x) < 0) ? -(x) : (x))                   // 取绝对值
#define GetSign(x) (((x) < 0) ? (-1) : ((x) > 0 ? 1 : 0)) // 取正负号

    // 电机模式枚举
    typedef enum
    {
        DJ_Disable = 0,  /* 关：transmit 0 current */
        DJ_RPM = 1,      /* 速度 mode */
        DJ_Position = 2, /* 位置 mode */
        DJ_Zero = 3,     /* 寻零 mode */
        DJ_Current = 4,  /* 电流/扭矩 */

    } DJMotor_mode_t;
    // 反馈及设定值结构体
    typedef struct
    {
        int16_t current_raw;  // 当前电流值
        int16_t speed_rpm;    // 反馈机械转速
        int16_t PulseRead;    // 读取到的原始脉冲
        int16_t PulseGap;     // 本次与上次的脉冲差
        int32_t PulseTotal;   // 累计总脉冲
        float angle_deg;      // 计算出来的机械角度
        float current_A;      // 换算后的电流（安培）
        int8_t temperature_C; // 电机温度（摄氏度）
    } DJMotorVal;

    typedef struct
    {
        uint16_t PulsePerRound;   // 编码器每转脉冲数，必填 8192！
        float Gear_ratio;         // 外部机械减速比（如果电机直接带轮子，填 1.0f）
        float Reduction_ratio;    // 电机自身减速比（必填，M3508是 19.0f，M2006是 36.0f）
        uint32_t ParamID;         // CAN receive ID base
        int16_t CurrentLimit_raw; // 电流最大限制值
    } DJMotorParam;

    // 电机限幅参数结构体（速度环/位置环的各种限制）
    typedef struct
    {
        uint8_t RPMLimitFlag;      // 是否启用速度限幅
        float SpeedRPMLimit;       // 速度上限
        uint8_t PosAngleLimitFlag; // 是否启用位置（角度）限幅
        float MinAngle_deq;        // 最小角度
        float MaxAngle_deq;        // 最大角度
        uint8_t PosRPMFlag;        // 位置模式内环速度限幅开关
        float PosRPMLimit;         // 位置模式内环速度上限
    } DJMotorLimit;

    // 电机本体结构体
    typedef struct
    {
        volatile bool Begin;    // true 运行 MODE;false 失能
        uint8_t ID;                       // 电机 ID (1~8)
        volatile DJMotor_mode_t MODE_Cur; // 当前运行模式
        DJMotorVal valSet;                // 设定值
        DJMotorVal valNow;                // 当前实际值
        DJMotorVal valPre;                // 上一次的值（用于算差值）
        PIDType velPID;                   // 速度 PID
        PIDType posPID;                   // 位置 PID
        DJMotorParam param;               // 电机物理档案
        DJMotorLimit limit;               // 电机限幅参数
        uint32_t lastRxTick;              // 最近一次收到反馈时的时间戳（HAL_GetTick）
        uint8_t rxLost;                   /* 失联标志：1=超过 CONNECT_TIMEOUT 未收到反馈，
                                             状态机会强制失能发 0 电流 */
    } DJMotor;

    // 全局电机数组
    extern DJMotor DJ_Motor[USE_DJNUM];

    extern CAN_HandleTypeDef hcan1; // 声明 CubeMX 生成的 CAN 句柄

    CAN_HandleTypeDef *DJmotor_GetCanHandle(void); // 获取 CAN 句柄（实现放 motor.c）

    void DJmotor_AngleCalculate(DJMotor *motor);                            // 计算脉冲与角度
    void DJMotor_Receive(CAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data); // 接收解析函数
    void DJmotor_CurrentTransmit(DJMotor *motor);                           // 封包发送函数
    void DJmotor_Init(void);                                                // 初始化电机
    void DJMotor_Func(void);                                                // 状态机主体（每1ms调用一次）

#ifdef __cplusplus
}
#endif
#endif /*__MOTOR_H__ */