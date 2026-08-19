#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C"
{
#endif

    // 定义PID类型枚举
    typedef enum
    {
        PIDINC = 0, // 增量式PID
        PIDPOS      // 位置式PID
    } PID_Mode;
    

    // PIDType结构体定义
    typedef struct
    {
        // PID核心参数
        float KP;
        float KI;
        float KD;

        // 控制目标值与当前实际值
        float SetVal; // 目标值
        float CurVal; // 当前实际值

        // 误差历史
        float err[3];

        // PID输出值
        float output;

        // PID工作模式
        PID_Mode mode;

        // 防止积分饱和
        float intgral;

    } PIDType;

    void PID_Reset(PIDType *pid);     // 把所有误差清零
    float PID_Caculate(PIDType *pid); // 算出误差，根据上次的误差算出这次油门应该增加多少（增量）

#ifdef __cplusplus
}
#endif
#endif /*__PID_H__ */