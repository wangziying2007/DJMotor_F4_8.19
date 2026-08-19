#include "motor.h"

//获取CAN句柄
FDCAN_HandleTypeDef *DJmotor_GetCanHandle(void)
{
    return &hfdcan2;// 以后如果换 CAN 接口，只需要改这一行，隔离底层硬件依赖
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
    if ((Rxheader.IdType != CAN_STANDARD_ID) ||
        (Rxheader.RxFrameType != CAN_DATA_FRAME) ||
        (Rxheader.Identifier < 0x201U) || (Rxheader.Identifier > 0x208U))
    {
        return;
    }

    uint8_t card_id = (uint8_t)(Rxheader.Identifier - 0x200U); /* 1..8 */

    /* Init 保证 ID = 索引 + 1, 直接索引免循环查找 */
    if (card_id > USE_DJNUM)
    {
        return;
    }

    DJMotor *motor = &DJ_Motor[card_id - 1U];

    motor->valNow.PulseRead = (int16_t)(((uint16_t)Rx_data[0] << 8) | Rx_data[1]);
    motor->valNow.speed_rpm = (int16_t)(((uint16_t)Rx_data[2] << 8) | Rx_data[3]);
    motor->valNow.current_raw = (int16_t)(((uint16_t)Rx_data[4] << 8) | Rx_data[5]);

    if (motor->param.Reduction_ratio == M3508_RATIO)              // C620 电调
    {
        motor->valNow.temperature_C = (int8_t)Rx_data[6];
        motor->valNow.current_A = (float)motor->valNow.current_raw * 0.0012207f;
    }
    else                                                          // C610 电调
    {
        motor->valNow.current_A = (float)motor->valNow.current_raw / 10000.0f * 10.0f;
    }

    motor->valNow.speed_rpm /= (motor->param.Gear_ratio * motor->param.Reduction_ratio);

    /* 记录最近收到反馈的时间，用于失联检测（DJMotor_Func 里超时刹车） */
    motor->lastRxTick = HAL_GetTick();
    motor->rxLost = 0;

    DJmotor_AngleCalculate(motor);
}

void EncodeS16Data(int16_t *src, uint8_t *dst)
{
    dst[0] = (uint8_t)(*src >> 8);
    dst[1] = (uint8_t)(*src & 0xFF);
}

void ChangeDataByte(uint8_t *byte1, uint8_t *byte2)
{
    uint8_t temp = *byte1;
    *byte1 = *byte2;
    *byte2 = temp;
}

void DJmotor_CurrentTransmit(DJMotor *motor)
{
    uint8_t tx_data[8] = {0};
    CAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tag = 0;

    /* 电流限幅由各模式函数负责，此处只打包发送 */
    tx_header.IdType = CAN_STANDARD_ID;
    tx_header.TxFrameType = CAN_DATA_FRAME;
    tx_header.DataLength = CAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = CAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = CAN_BRS_OFF;
    tx_header.FDFormat = CAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = CAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    /* 编号 1~4 -> 0x200 帧，5~8 -> 0x1FF 帧；每个电机占 2 字节 */
    if (motor->ID <= 4U)
    {
        tx_header.Identifier = 0x200U;
        tag = (uint8_t)((motor->ID - 1U) * 2U);
    }
    else
    {
        tx_header.Identifier = 0x1FFU;
        tag = (uint8_t)((motor->ID - 5U) * 2U);
    }

    /* C620/C610 电流帧字节序为 [电流低8位, 电流高8位]。
       EncodeS16Data 先按 [高,低] 写入，ChangeDataByte 再翻成 [低,高]，最终正确。 */
    EncodeS16Data(&motor->valSet.current_raw, &tx_data[tag]);
    ChangeDataByte(&tx_data[tag], &tx_data[tag + 1U]);

    HAL_CAN_AddMessageToTxFifoQ(DJmotor_GetCanHandle(), &tx_header, tx_data);
}

// 辅助限幅函数
float ClampPeak(float val, float limit)
{
    if (val > limit)
        return limit;
    if (val < -limit)
        return -limit;
    return val;
}

// 辅助范围限幅函数
float Clamp(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

void DJMotor_Func(void)
{
    for (uint32_t i = 0; i < USE_DJNUM; i++)
    {
        DJMotor *motor = &DJ_Motor[0];

        /* 失联检测：超过 CONNECT_TIMEOUT 未收到反馈，强制失能刹车（发 0 电流） */
        uint32_t connect_timeout = 100U;  /* 100ms */
        if (motor->MODE_Cur != DJ_Disable &&
            ((uint32_t)(HAL_GetTick() - motor->lastRxTick) > connect_timeout))
        {
            motor->lastRxTick = HAL_GetTick(); /* 减速刹车期间避免每次循环都被判定打进失能分支 */
            motor->rxLost = 1;
            motor->valSet.current_raw = 0;
            DJmotor_CurrentTransmit(motor);
            continue;
        }

        // 如果电机失能
        if (motor->MODE_Cur == DJ_Disable)
        {
            motor->valSet.current_raw = 0;
            DJmotor_CurrentTransmit(motor);
            continue; // 当前电机处理结束，跳到下一个循环
        }

        // 速度模式
        if (motor->MODE_Cur == DJ_RPM)
        {
            // 1. 把目标速度乘以减速比，转化为电机轴本身的目标速度
            float target_rpm = (float)motor->valSet.speed_rpm * motor->param.Gear_ratio * motor->param.Reduction_ratio;

            // 2. 如果开启了速度限幅，进行限制
            if (motor->limit.RPMLimitFlag)
            { // 注意：如果你没有定义 limit 结构体，这句可暂时注释
                target_rpm = ClampPeak(target_rpm, motor->limit.SpeedRPMLimit);
            }

            // 3. 更新 PID 的输入和输出
            motor->velPID.SetVal = target_rpm;
            motor->velPID.CurVal = (float)motor->valNow.speed_rpm * motor->param.Gear_ratio * motor->param.Reduction_ratio;

            // 4. 调用 PID 计算增量
            motor->valSet.current_raw += PID_Caculate(&motor->velPID);

            // 5. 电流限幅保护
            motor->valSet.current_raw = (int16_t)ClampPeak(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
        }
        else if (motor->MODE_Cur == DJ_Position)
        {
            // 位置模式

            // === 外环：位置环 ===
            // 1. 目标角度转换到目标脉冲数
            motor->valSet.PulseTotal = (int32_t)(motor->valSet.angle_deg * motor->param.Gear_ratio *
                                                 motor->param.Reduction_ratio * motor->param.PulsePerRound / 360.0f);

            motor->posPID.SetVal = (float)motor->valSet.PulseTotal;
            motor->posPID.CurVal = (float)motor->valNow.PulseTotal; // 当前实际脉冲

            // 2. 如果开启了角度限幅
            if (motor->limit.PosAngleLimitFlag)
            {
                motor->posPID.SetVal = Clamp(motor->posPID.SetVal, motor->limit.MinAngle_deq * motor->param.Gear_ratio * motor->param.Reduction_ratio * motor->param.PulsePerRound / 360.0f,
                                             motor->limit.MaxAngle_deq * motor->param.Gear_ratio * motor->param.Reduction_ratio * motor->param.PulsePerRound / 360.0f);
            }

            // === 内环：速度环（把位置环的输出，作为速度环的目标值） ===
            // 3. 位置环 PID 计算
            motor->velPID.SetVal = PID_Caculate(&motor->posPID);
            motor->velPID.CurVal = (float)motor->valNow.speed_rpm * motor->param.Gear_ratio * motor->param.Reduction_ratio;

            // 4. 如果开启了速度限幅（内环的限幅）
            if (motor->limit.PosRPMFlag)
            {
                motor->velPID.SetVal = ClampPeak(motor->velPID.SetVal, motor->limit.PosRPMLimit);
            }

            // 5. 调用速度环 PID 计算增量电流
            motor->valSet.current_raw += PID_Caculate(&motor->velPID);

            // 6. 最终限幅输出电流
            motor->valSet.current_raw = (int16_t)ClampPeak(motor->valSet.current_raw, motor->param.CurrentLimit_raw);
        }
        else
        {
            // 其他模式（比如没设置）默认发 0 电流
            motor->valSet.current_raw = 0;
        }

        // 最后一步：无论什么模式，都要调用发送函数把计算好的电流发给电调
        DJmotor_CurrentTransmit(motor);
    }
}