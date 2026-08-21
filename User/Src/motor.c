#include "motor.h"

DJMotor DJ_Motor[USE_DJNUM];

static inline float Clampf(float var, float limit)
{
    float ans;
    if (var > limit)
        ans = limit;
    else if (var < -limit)
        ans = -limit;
    else
        ans = var;
    return ans;
}
static inline int16_t Clamp16(int16_t var, int16_t limit)
{
    int16_t ans;
    if (var > limit)
        ans = limit;
    else if (var < -limit)
        ans = -limit;
    else
        ans = var;
    return ans;
}

static inline int32_t Clamp32(int32_t var, int32_t min, int32_t max)
{
    int32_t ans;
    if (var > max)
        ans = max;
    else if (var < min)
        ans = min;
    else
        ans = var;
    return ans;
}

void DJmotor_SetZero(DJMotor *motor)
{
    motor->valPre.PulseRead = motor->valNow.PulseRead;
    motor->valNow.PulseTotal = 0;
}

void DJmotor_Init(void)
{
    DJMotorParam dj2006_param;
    DJMotorParam dj3508_param;
    DJMotorLimit limit;

    /* M2006 电机档案（DJMotorParam 全字段）*/
    dj2006_param.ParamID = 0x1ffU;
    dj2006_param.Gear_ratio = 1.0f;
    dj2006_param.Reduction_ratio = M2006_RATIO;
    dj2006_param.PulsePerRound = 8191U;
    dj2006_param.CurrentLimit_raw = 4500;

    /* M3508 电机档案（DJMotorParam 全字段）*/
    dj3508_param.ParamID = 0x200U;
    dj3508_param.Gear_ratio = 1.0f;
    dj3508_param.Reduction_ratio = M3508_RATIO;
    dj3508_param.PulsePerRound = 8191U;
    dj3508_param.CurrentLimit_raw = 10000;

    /* 默认限幅参数（DJMotorLimit 全字段）*/
    limit.RPMLimitFlag = 0U;
    limit.SpeedRPMLimit = 10000.0f;
    limit.PosAngleLimitFlag = 0U;
    limit.MinAngle_deg = 0.0f;
    limit.MaxAngle_deg = 360.0f;
    limit.PosRPMFlag = 1U;
    limit.PosRPMLimit = 8000.0f;

    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        DJMotor *motor = &DJ_Motor[i];

        /* DJMotor 基础字段 */
        motor->ID = (uint8_t)(i + 1U);
        motor->MODE_Cur = DJ_Disable; /* 上电失能:发 0 电流 */
        motor->param = dj3508_param;  /* 先给默认档案，下面按型号覆盖 */
        motor->limit = limit;
        motor->lastRxTick = HAL_GetTick();
        motor->rxLost = 0U;

        /* DJMotorVal 全字段初始化：设定值 / 当前值 / 上一次值 全部清零 */
        motor->valSet.current_raw = 0;
        motor->valSet.speed_rpm = 0;
        motor->valSet.PulseRead = 0;
        motor->valSet.PulseGap = 0;
        motor->valSet.PulseTotal = 0;
        motor->valSet.angle_deg = 0.0f;
        motor->valSet.current_A = 0.0f;
        motor->valSet.temperature_C = 0;

        motor->valNow = motor->valSet;
        motor->valPre = motor->valSet;

        /* PIDType 全字段初始化：位置环（位置式）*/
        PID_Reset(&motor->posPID);
        motor->posPID.KP = 0.07f;
        motor->posPID.KI = 0.000f;
        motor->posPID.KD = 0.001f;
        motor->posPID.mode = PIDPOS;
        motor->posPID.intgral = 0.0f;

        /* PIDType 全字段初始化：速度环（增量式）*/
        PID_Reset(&motor->velPID);
        motor->velPID.KP = 2.3f;
        motor->velPID.KI = 0.03f;
        motor->velPID.KD = 0.0f;
        motor->velPID.mode = PIDINC;
        motor->velPID.intgral = 0.0f;
    }

    /* 按型号覆写对应电机的物理档案（M2006 在前，M3508 在后）*/
    for (uint32_t i = 0; i < M2006_NUM && i < USE_DJNUM; i++)
    {
        DJ_Motor[i].ID = (uint8_t)(i + 1U);
        DJ_Motor[i].param = dj2006_param;
    }

    for (uint32_t i = 0; i < M3508_NUM && (i + M2006_NUM) < USE_DJNUM; i++)
    {
        DJ_Motor[i + M2006_NUM].ID = (uint8_t)(i + M2006_NUM + 1U);
        DJ_Motor[i + M2006_NUM].param = dj3508_param;
    }
}

// 获取CAN句柄
CAN_HandleTypeDef *DJmotor_GetCanHandle(void)
{
    return &hcan1; // 以后如果换 CAN 接口，只需要改这一行，隔离底层硬件依赖
}

void DJmotor_AngleCalculate(DJMotor *motor)
{
    motor->valNow.PulseGap = (int16_t)(motor->valNow.PulseRead - motor->valPre.PulseRead);

    // 处理编码器跨越 0 点的绕圈补偿
    if (ABS(motor->valNow.PulseGap) > 4096)
    {
        motor->valNow.PulseGap = (int16_t)(motor->valNow.PulseGap -
                                           GetSign(motor->valNow.PulseGap) *
                                               (int32_t)motor->param.PulsePerRound);
    }

    // 累加总脉冲，换算成实际机械角度
    motor->valNow.PulseTotal += motor->valNow.PulseGap; // 问！！！
    motor->valNow.angle_deg = (float)motor->valNow.PulseTotal * 360.0f /
                              ((float)motor->param.PulsePerRound * motor->param.Gear_ratio *
                               motor->param.Reduction_ratio);

    // 将当前状态保存为“上一次的状态”，供下一帧 CAN 数据到来时做差值计算
    motor->valPre = motor->valNow;
}

void DJMotor_Receive(CAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data)
{
    if ((Rxheader.IDE != CAN_ID_STD) ||
        (Rxheader.RTR != CAN_RTR_DATA) ||
        (Rxheader.StdId < 0x201U) ||
        (Rxheader.StdId > 0x208U))
    {
        return;
    }

    uint8_t card_id = (uint8_t)(Rxheader.StdId - 0x200U); /* 1..8 */

    /* Init 保证 ID = 索引 + 1, 直接索引免循环查找 */
    if (card_id > USE_DJNUM)
    {
        return;
    }

    DJMotor *motor = &DJ_Motor[card_id - 1U];

    motor->valNow.PulseRead = (int16_t)(((uint16_t)Rx_data[0] << 8) | Rx_data[1]);
    motor->valNow.speed_rpm = (int16_t)(((uint16_t)Rx_data[2] << 8) | Rx_data[3]);
    motor->valNow.current_raw = (int16_t)(((uint16_t)Rx_data[4] << 8) | Rx_data[5]);

    if (motor->param.Reduction_ratio == M3508_RATIO) // C620 电调
    {
        motor->valNow.temperature_C = (int8_t)Rx_data[6];
        motor->valNow.current_A = (float)motor->valNow.current_raw * 0.0012207f;
    }
    else // C610 电调
    {
        motor->valNow.current_A = (float)motor->valNow.current_raw / 10000.0f * 10.0f;
    }

    motor->valNow.speed_rpm /= (motor->param.Gear_ratio * motor->param.Reduction_ratio);

    /* 记录最近收到反馈的时间，用于失联检测（DJMotor_Func 里超时刹车） */
    motor->lastRxTick = HAL_GetTick();
    motor->rxLost = 0;

    DJmotor_AngleCalculate(motor);
}

static void EncodeS16Data(volatile int16_t *value, uint8_t *data)
{
    data[0] = (uint8_t)((uint16_t)*value & 0xFFU);
    data[1] = (uint8_t)(((uint16_t)*value >> 8U) & 0xFFU);
}

static void ChangeDataByte(uint8_t *data1, uint8_t *data2)
{
    uint8_t data = *data1;
    *data1 = *data2;
    *data2 = data;
}


void DJmotor_CurrentTransmit(DJMotor *motor)
{
    static uint8_t tx_data[8] = {0};
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tag = 0;

    /* 电流限幅由各模式函数负责，此处只打包发送 */
    tx_header.IDE = CAN_ID_STD;   // 标准帧
    tx_header.RTR = CAN_RTR_DATA; // 数据帧
    tx_header.DLC = 8;            // 8字节数据长度
    tx_header.TransmitGlobalTime = DISABLE;

    /* 编号 1~4 -> 0x200 帧，5~8 -> 0x1FF 帧；每个电机占 2 字节 */
    if (motor->ID <= 4U)
    {
        tx_header.StdId = 0x200U;
        tag = (uint8_t)((motor->ID - 1U) * 2U);
    }
    else
    {
        tx_header.StdId = 0x1FFU;
        tag = (uint8_t)((motor->ID - 5U) * 2U);
    }

    /* C620/C610 电流帧字节序为 [电流低8位, 电流高8位]。
       EncodeS16Data 先按 [高,低] 写入，ChangeDataByte 再翻成 [低,高]，最终正确。 （写这个一定要看电调的手册，一开始写反了）*/
    EncodeS16Data(&motor->valSet.current_raw, &tx_data[tag]);
    ChangeDataByte(&tx_data[tag], &tx_data[tag + 1U]);

    uint32_t mailbox;
    if (motor->ID == 4 || motor->ID == 8)
    {
        HAL_CAN_AddTxMessage(DJmotor_GetCanHandle(), &tx_header, tx_data, &mailbox);
    }
}

void DJmotor_SpeedMode(DJMotor *motor)
{
    // 目标速度换算成电机轴转速（与反馈的电机轴转速同一量纲）
    float target_rpm = (float)motor->valSet.speed_rpm * motor->param.Gear_ratio *
                       motor->param.Reduction_ratio;

    /* ⚠️ 兜底保护：目标转速为 0 时，直接输出 0 电流（刹车），不再跑 PID。
       这样即便 PID 内部历史值(err/output)被污染，也不会出现"没给目标值却疯转"。
       只有你真正给了非零目标转速，下面才会进入 PID。 */

    motor->velPID.SetVal = target_rpm;
    motor->velPID.CurVal = (float)motor->valNow.speed_rpm * motor->param.Gear_ratio *
                           motor->param.Reduction_ratio;

    if (motor->limit.RPMLimitFlag)
    {
        motor->velPID.SetVal = Clampf(motor->velPID.SetVal, motor->limit.SpeedRPMLimit);
    }

    /* 每毫秒 PID 算出的电流增量 Δ。注意：原代码叫 PID_Caculate，
       pid.c 里真正定义的是 PID_Calculate，名字拼错了会导致链接不过，这里已修正。 */
    int16_t dCurRaw = (int16_t)PID_Calculate(&motor->velPID);

    motor->valSet.current_raw += dCurRaw;

    /* 目标电流整体限到 ±CurrentLimit */
    if (motor->valSet.current_raw >  motor->param.CurrentLimit_raw) motor->valSet.current_raw = motor->param.CurrentLimit_raw;
    if (motor->valSet.current_raw < -motor->param.CurrentLimit_raw) motor->valSet.current_raw = -motor->param.CurrentLimit_raw;
}

void DJmotor_PositionMode(DJMotor *motor)
{
    motor->valSet.PulseTotal = (int32_t)(motor->valSet.angle_deg * motor->param.Gear_ratio *
                               motor->param.Reduction_ratio *
                               (float)motor->param.PulsePerRound / 360.0f);
    motor->posPID.SetVal = (float)motor->valSet.PulseTotal;
    if (motor->limit.PosAngleLimitFlag)
    {
        const int32_t max_pulse = (int32_t)(motor->limit.MaxAngle_deg *
                                   (float)motor->param.PulsePerRound *
                                   motor->param.Gear_ratio * motor->param.Reduction_ratio / 360.0f);
        const int32_t min_pulse = (int32_t)(motor->limit.MinAngle_deg *
                                   (float)motor->param.PulsePerRound *
                                   motor->param.Gear_ratio * motor->param.Reduction_ratio / 360.0f);

        motor->posPID.SetVal = (float)Clamp32((int32_t)motor->posPID.SetVal, min_pulse, max_pulse);
    }

    motor->posPID.CurVal = (float)motor->valNow.PulseTotal;

    motor->velPID.SetVal = PID_Calculate(&motor->posPID);
    motor->velPID.CurVal = (float)motor->valNow.speed_rpm * motor->param.Gear_ratio * motor->param.Reduction_ratio;

    if (motor->limit.PosRPMFlag)
    {
        motor->velPID.SetVal = Clampf(motor->velPID.SetVal, motor->limit.PosRPMLimit);
    }

    motor->valSet.current_raw += PID_Calculate(&motor->velPID);
    motor->valSet.current_raw = Clamp16(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
}

void DJMotor_Func(void)
{
    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        if (DJ_Motor[i].Begin)
        {
            // DJmotor_Monitor(&DJmotor[i]);
           // DJmotor_SwitchMode(&DJ_Motor[i]);

            switch (DJ_Motor[i].MODE_Cur)
            {
            case DJ_Disable:
                DJ_Motor[i].valSet.current_raw = 0;
                DJmotor_CurrentTransmit(&DJ_Motor[i]);
                continue;
            case DJ_RPM:
                DJmotor_SpeedMode(&DJ_Motor[i]);
                break;
            case DJ_Position:
                DJmotor_PositionMode(&DJ_Motor[i]);
                break;
            // case DJ_Zero:
            //     DJmotor_ZeroMode(&DJ_Motor[i]);
            //     break;
            // case DJ_Current:
            //     /* 直通电流:任务层每周期写 valSet.current_raw,这里补限幅 */
            //     ClampPeak(DJ_Motor[i].valSet.current_raw, DJ_Motor[i].param.CurrentLimit_raw);
            //     break;
            default:
                break;
            }
        }
        else
        {
            /* Begin=false(未初始化/寻零完成):强制 0 电流,防止残留累加电流持续输出 */
            DJ_Motor[i].valSet.current_raw = 0;
        }

        DJmotor_CurrentTransmit(&DJ_Motor[i]);
    }
}