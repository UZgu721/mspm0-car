/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include "ti_msp_dl_config.h"
#include "KEY.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "huidu.h"
#include "adc_dma.h"

uint8_t i = 0;
extern struct Bkeys bkeys[3];
int32_t vel_left = 0, vel_right = 0;
uint8_t time_40ms = 0;
extern PID_t motorA, motorB;
uint8_t pid_flag = 0, huidu_pid_flag = 0;
extern uint8_t turn_count;
extern uint8_t lap_count;
extern uint8_t target_lap;

int main(void)
{
    SYSCFG_DL_init();
    Encoder_Init();

    /* 灰度传感器校准初始化 */
    Huidu_Sensor_Init();

    /* 启动ADC连续转换（repeat模式） */
    DL_ADC12_startConversion(ADC12_0_INST);

    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    OLED_Init();

    while (1) {
        /* 开机自动校准（放在白底上），完成后自动进入正常显示 */
        if (!Huidu_AutoCalibrate()) {
            memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
            OLED_ShowString(20, 24, (uint8_t *)"CALIB...");
            OLED_Refresh_Gram();
            continue;
        }

        /* 灰度传感器持续采集（ADC+DMA，在后台更新） */
        Huidu_Sensor_Task();

        memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));

        /* 第1行：左右轮转速（10ms编码器脉冲增量） */
        OLED_ShowString(0, 0, (uint8_t *)"L:");
        OLED_ShowSignedNum(12, 0, vel_left, 6, 12);
        OLED_ShowString(54, 0, (uint8_t *)"R:");
        OLED_ShowSignedNum(66, 0, vel_right, 6, 12);

        /* 第2行：8路灰度状态 bit7..bit0 */
        {
            uint8_t g = Read_HuiDu();
            uint8_t buf[12];
            buf[0] = 'G'; buf[1] = ':';
            for (int i = 0; i < 8; i++)
                buf[2 + i] = (g & (0x80 >> i)) ? '1' : '0';
            buf[10] = '\0';
            OLED_ShowString(0, 16, buf);
        }

        /* 第4行：目标圈数 */
        OLED_ShowString(0, 48, (uint8_t *)"LAP:");
        OLED_ShowNumber(30, 48, target_lap, 2, 12);

        OLED_Refresh_Gram();
    }
}

/* 10ms定时中断 */
void TIMG0_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_LOAD) {

        vel_left  = Get_encoder_left();
        vel_right = Get_encoder_right();

        if (huidu_pid_flag == 1) {
            HuiDu_PID();
        }
        key_serv_double();

        if (bkeys[1].short_flag == 1) {
            target_lap++;
            if (target_lap > 5) {
                target_lap = 0;
            }
            bkeys[1].short_flag = 0;
        }
        if (bkeys[1].long_flag == 1) {
            huidu_pid_flag = 1;
            bkeys[1].long_flag = 0;
        }
    }
}
