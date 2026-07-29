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
#include "icm42688.h"
#include "ti_msp_dl_config.h"
#include "KEY.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "huidu.h"
#include "imu.h"
#include "ahrs.h"
#include "adc_dma.h"

uint8_t i = 0;
extern struct Bkeys bkeys[3];
int32_t vel_left = 0, vel_right = 0;
uint8_t time_40ms = 0;
extern PID_t motorA, motorB;
uint8_t pid_flag = 0, huidu_pid_flag = 0;
volatile uint32_t sys_10ms_tick = 0;  /* 10ms 定时器计数 */
volatile uint8_t  bmi_ok       = 0;  /* BMI088 就绪标志（ISR访问） */
volatile uint8_t  oled_debug_page = 0;
extern uint8_t turn_count;
extern uint8_t lap_count;
extern uint8_t target_lap;

/* UART1 / HC-05 bilateral protocol: <Y:+012.3>
 * Each node periodically sends its own yaw and uses the same parser to save
 * the peer yaw.  Incoming data is diagnostic-only and never drives motors. */
#define BLUETOOTH_YAW_PERIOD_TICKS  100U /* 1 s */
#define BLUETOOTH_PEER_TIMEOUT_TICKS 500U /* 5 s */
#define BLUETOOTH_YAW_BODY_LENGTH   8U
#define BLUETOOTH_RX_PACKET_LENGTH   18U
#define BLUETOOTH_RX_QUEUE_SIZE      32U
#define BLUETOOTH_RX_QUEUE_MASK      (BLUETOOTH_RX_QUEUE_SIZE - 1U)

static uint8_t bluetooth_rx_body[BLUETOOTH_YAW_BODY_LENGTH];
static uint8_t bluetooth_rx_length;
static uint8_t bluetooth_receiving;
/* Raw packet monitor: accepts ordinary phone serial-assistant text too. */
static uint8_t bluetooth_rx_packet[BLUETOOTH_RX_PACKET_LENGTH + 1U] = "---";
static uint8_t bluetooth_rx_packet_length;
static uint8_t bluetooth_rx_packet_complete;
static float bluetooth_peer_yaw;
static uint8_t bluetooth_peer_yaw_valid;
static uint32_t bluetooth_peer_tick;
static uint32_t bluetooth_last_tx_tick;
static uint8_t bluetooth_tx_frame[10];
static uint8_t bluetooth_tx_index;
static uint8_t bluetooth_tx_length;
/* UART ISR writes this queue; foreground code alone parses the frame. */
static volatile uint8_t bluetooth_rx_queue[BLUETOOTH_RX_QUEUE_SIZE];
static volatile uint8_t bluetooth_rx_write_index;
static volatile uint8_t bluetooth_rx_read_index;
static volatile uint16_t bluetooth_rx_overflow_count;
static volatile uint16_t bluetooth_uart_irq_count;
static volatile uint16_t bluetooth_uart_byte_count;

static void Bluetooth_ParseYawFrame(void)
{
    uint32_t magnitude;

    if ((bluetooth_rx_length != BLUETOOTH_YAW_BODY_LENGTH) ||
        (bluetooth_rx_body[0] != 'Y') ||
        (bluetooth_rx_body[1] != ':') ||
        ((bluetooth_rx_body[2] != '+') && (bluetooth_rx_body[2] != '-')) ||
        (bluetooth_rx_body[6] != '.') ||
        (bluetooth_rx_body[3] < '0') || (bluetooth_rx_body[3] > '9') ||
        (bluetooth_rx_body[4] < '0') || (bluetooth_rx_body[4] > '9') ||
        (bluetooth_rx_body[5] < '0') || (bluetooth_rx_body[5] > '9') ||
        (bluetooth_rx_body[7] < '0') || (bluetooth_rx_body[7] > '9')) {
        return;
    }

    magnitude = (uint32_t)(bluetooth_rx_body[3] - '0') * 1000U +
                (uint32_t)(bluetooth_rx_body[4] - '0') * 100U +
                (uint32_t)(bluetooth_rx_body[5] - '0') * 10U +
                (uint32_t)(bluetooth_rx_body[7] - '0');
    bluetooth_peer_yaw = (float)magnitude * 0.1f;
    if (bluetooth_rx_body[2] == '-') {
        bluetooth_peer_yaw = -bluetooth_peer_yaw;
    }
    bluetooth_peer_yaw_valid = 1U;
    bluetooth_peer_tick = sys_10ms_tick;
}

static void Bluetooth_HandleByte(uint8_t byte)
{
    if (byte == '<') {
        bluetooth_rx_length = 0U;
        bluetooth_receiving = 1U;
    } else if (byte == '>') {
        if (bluetooth_receiving != 0U) {
            Bluetooth_ParseYawFrame();
        }
        bluetooth_rx_length = 0U;
        bluetooth_receiving = 0U;
    } else if (bluetooth_receiving != 0U) {
        if (bluetooth_rx_length < sizeof(bluetooth_rx_body)) {
            bluetooth_rx_body[bluetooth_rx_length++] = byte;
        } else {
            /* Oversize/noisy frame: discard it and wait for the next '<'. */
            bluetooth_rx_length = 0U;
            bluetooth_receiving = 0U;
        }
    }
}

/* Store every printable byte independently of the yaw-frame parser.
 * A '<' starts a new framed message; CR/LF terminates ordinary text. */
static void Bluetooth_StoreRawByte(uint8_t byte)
{
    if (byte == '<') {
        bluetooth_rx_packet_length = 0U;
        bluetooth_rx_packet[0] = '\0';
        bluetooth_rx_packet_complete = 0U;
    }

    if ((byte == '\r') || (byte == '\n')) {
        if (bluetooth_rx_packet_length != 0U) {
            bluetooth_rx_packet_complete = 1U;
        }
        return;
    }

    if ((byte >= 0x20U) && (byte <= 0x7EU) &&
        (bluetooth_rx_packet_length < BLUETOOTH_RX_PACKET_LENGTH)) {
        if (bluetooth_rx_packet_complete != 0U) {
            bluetooth_rx_packet_length = 0U;
            bluetooth_rx_packet[0] = '\0';
            bluetooth_rx_packet_complete = 0U;
        }
        bluetooth_rx_packet[bluetooth_rx_packet_length++] = byte;
        bluetooth_rx_packet[bluetooth_rx_packet_length] = '\0';
    }
}

static void Bluetooth_QueueYaw(float yaw)
{
    uint32_t magnitude;

    magnitude = (uint32_t)((yaw < 0.0f ? -yaw : yaw) * 10.0f + 0.5f);
    if (magnitude > 9999U) {
        magnitude = 9999U;
    }

    bluetooth_tx_frame[0] = '<';
    bluetooth_tx_frame[1] = 'Y';
    bluetooth_tx_frame[2] = ':';
    bluetooth_tx_frame[3] = (yaw < 0.0f) ? '-' : '+';
    bluetooth_tx_frame[4] = (uint8_t)('0' + (magnitude / 1000U));
    bluetooth_tx_frame[5] = (uint8_t)('0' + (magnitude / 100U) % 10U);
    bluetooth_tx_frame[6] = (uint8_t)('0' + (magnitude / 10U) % 10U);
    bluetooth_tx_frame[7] = '.';
    bluetooth_tx_frame[8] = (uint8_t)('0' + magnitude % 10U);
    bluetooth_tx_frame[9] = '>';
    bluetooth_tx_index = 0U;
    bluetooth_tx_length = sizeof(bluetooth_tx_frame);
}

static void Bluetooth_Task(void)
{
    uint32_t now = sys_10ms_tick;
    uint8_t byte;

    while (bluetooth_rx_read_index != bluetooth_rx_write_index) {
        byte = bluetooth_rx_queue[bluetooth_rx_read_index];
        bluetooth_rx_read_index =
            (bluetooth_rx_read_index + 1U) & BLUETOOTH_RX_QUEUE_MASK;
        Bluetooth_StoreRawByte(byte);
        Bluetooth_HandleByte(byte);
    }

    /* Advance the TX frame only after the peripheral reports space. */
    if (bluetooth_tx_index < bluetooth_tx_length) {
        if (DL_UART_Main_isTXFIFOFull(UART_1_INST) == false) {
            DL_UART_Main_transmitData(UART_1_INST,
                                      bluetooth_tx_frame[bluetooth_tx_index]);
            ++bluetooth_tx_index;
        }
        return;
    }

    if ((bmi_ok != 0U) &&
        ((uint32_t)(now - bluetooth_last_tx_tick) >= BLUETOOTH_YAW_PERIOD_TICKS)) {
        bluetooth_last_tx_tick = now;
        Bluetooth_QueueYaw(AHRS_GetYaw());
    }
}

static uint8_t Bluetooth_HasFreshPeerYaw(void)
{
    return (bluetooth_peer_yaw_valid != 0U) &&
           ((uint32_t)(sys_10ms_tick - bluetooth_peer_tick) <=
            BLUETOOTH_PEER_TIMEOUT_TICKS);
}

static void OLED_ShowBluetoothReceive(void)
{
    OLED_ShowString(0, 48, (uint8_t *)"RX:");

    if (bluetooth_rx_packet_length != 0U) {
        OLED_ShowString(18, 48, bluetooth_rx_packet);
    } else {
        OLED_ShowString(18, 48, (uint8_t *)"---");
        OLED_ShowString(42, 48, (uint8_t *)"I:");
        OLED_ShowNumber(54, 48, bluetooth_uart_irq_count % 100U, 2, 12);
        OLED_ShowString(72, 48, (uint8_t *)"B:");
        OLED_ShowNumber(84, 48, bluetooth_uart_byte_count % 100U, 2, 12);
    }
}

static void OLED_ShowFixed(uint8_t x, uint8_t y, float value,
                           uint8_t integer_digits, uint8_t fraction_digits)
{
    char text[14];
    uint32_t scale = 1U;
    uint32_t whole_limit = 1U;
    uint32_t magnitude;
    uint32_t divisor;
    uint8_t index = 0U;
    uint8_t i;

    for (i = 0U; i < fraction_digits; ++i) scale *= 10U;
    for (i = 0U; i < integer_digits; ++i) whole_limit *= 10U;

    text[index++] = (value < 0.0f) ? '-' : '+';
    magnitude = (uint32_t)((value < 0.0f ? -value : value) * (float)scale + 0.5f);
    if (magnitude >= whole_limit * scale) magnitude = whole_limit * scale - 1U;

    divisor = scale * (whole_limit / 10U);
    for (i = 0U; i < integer_digits; ++i) {
        text[index++] = (char)('0' + (magnitude / divisor) % 10U);
        divisor /= 10U;
    }
    if (fraction_digits != 0U) {
        text[index++] = '.';
        divisor = scale / 10U;
        for (i = 0U; i < fraction_digits; ++i) {
            text[index++] = (char)('0' + (magnitude / divisor) % 10U);
            divisor /= 10U;
        }
    }
    text[index] = '\0';
    OLED_ShowString(x, y, (uint8_t *)text);
}

static void OLED_DrawDebugPage(void)
{
    memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
    OLED_ShowString(0, 0, (uint8_t *)"Y:");
    OLED_ShowFixed(16, 0, AHRS_GetYaw(), 3U, 1U);

    OLED_ShowString(0, 16, (uint8_t *)"GZ:");
    OLED_ShowFixed(24, 16, IMU_GetGyroZ(), 1U, 2U);
    OLED_ShowString(72, 16, (uint8_t *)"B:");
    OLED_ShowFixed(88, 16, IMU_GetGyroZBias(), 1U, 2U);

    OLED_ShowString(0, 32, (uint8_t *)"DT:");
    OLED_ShowNumber(24, 32, IMU_GetLastDtMs(), 3, 12);
    OLED_ShowString(48, 32, (uint8_t *)"CAL:");
    OLED_ShowString(80, 32, (uint8_t *)(IMU_IsCalibrated() ? "OK" : "--"));

    OLED_ShowBluetoothReceive();
}

int main(void)
{
    SYSCFG_DL_init();

    /* Wait for external modules and their I2C supply rails to stabilize. */
    delay_ms(5000U);

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
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

    OLED_Init();

    while (1) {
        /* 开机自动校准（放在白底上），完成后自动进入正常显示 */
        if (!Huidu_AutoCalibrate()) {
            memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));
            OLED_ShowString(20, 24, (uint8_t *)"CALIB...");
            OLED_Refresh_Gram();
            continue;
        }

        /* BMI088 初始化（仅尝试一次，失败则跳过） */
        {
            static uint8_t bmi_attempted = 0U;
            static uint8_t bmi_scan_done = 0U;
            static uint8_t bmi_scan_count = 0U;
            static uint8_t bmi_scan_addresses[3] = {0U, 0U, 0U};
            static uint8_t bmi_id_status = 0U;
            static uint8_t bmi_accel_id = 0U;
            static uint8_t bmi_gyro_id = 0U;
            static uint8_t bmi_imu_error = 0U;
            static uint8_t bmi_bus_error = 0U;
            static uint8_t bmi_init_stage = 0U;
            static uint32_t bmi_retry_tick = 0U;
            static uint32_t bmi_last_tick = 0U;
            uint32_t bmi_now = sys_10ms_tick;

            if (!bmi_ok && (!bmi_attempted ||
                            (uint32_t)(bmi_now - bmi_retry_tick) >= 100U)) {
                bmi_attempted = 1U;
                bmi_retry_tick = bmi_now;
                bmi_ok = IMU_InitAndCalibrate();
                if (bmi_ok) {
                    AHRS_Init(0.0f, 0.0f);
                    bmi_last_tick = sys_10ms_tick;
                } else if (!bmi_scan_done) {
                    /* Preserve the real cause before the diagnostic reads
                     * clear the I2C driver's last-error state. */
                    bmi_imu_error = IMU_GetInitError();
                    bmi_bus_error = BMI088_GetLastError();
                    bmi_init_stage = BMI088_GetInitStage();
                    bmi_scan_count = BMI088_ScanI2C(bmi_scan_addresses,
                                                    sizeof(bmi_scan_addresses));
                    bmi_id_status = BMI088_ReadRawIds(&bmi_accel_id,
                                                       &bmi_gyro_id);
                    bmi_scan_done = 1U;
                }
            }

        /* 灰度传感器持续采集（ADC+DMA，在后台更新） */
        Huidu_Sensor_Task();

        /* IMU has priority over all OLED drawing. */
        if (bmi_ok) {
            uint32_t now = sys_10ms_tick;
            int32_t diff = (int32_t)(now - bmi_last_tick);
            if (diff > 0) {
                IMU_Update((float)diff * 0.01f, huidu_pid_flag == 0U);
                bmi_last_tick = now;
            }
        }

        Bluetooth_Task();

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

        /* 第3行：互补滤波 Yaw */
        if (bmi_ok) {
            OLED_ShowString(0, 32, (uint8_t *)"YAW:");
            OLED_ShowSignedNum(30, 32, (int32_t)AHRS_GetYaw(), 5, 12);
        } else {
            uint8_t i;
            uint8_t pos = 0U;
            uint8_t shown = bmi_scan_count;
            uint8_t text[16] = "I2C:";
            static const uint8_t hex[] = "0123456789ABCDEF";

            if (!bmi_scan_done) {
                OLED_ShowString(0, 32, (uint8_t *)"I2C:SCAN");
            } else if (bmi_id_status != 0U) {
                uint8_t id_text[] = "E0B0S0 1E0F";

                id_text[1] = (uint8_t)('0' + bmi_imu_error);
                id_text[3] = (uint8_t)('0' + bmi_bus_error);
                id_text[5] = (uint8_t)('0' + bmi_init_stage);

                if ((bmi_id_status & 0x01U) != 0U) {
                    id_text[7] = hex[(bmi_accel_id >> 4) & 0x0FU];
                    id_text[8] = hex[bmi_accel_id & 0x0FU];
                }
                if ((bmi_id_status & 0x02U) != 0U) {
                    id_text[9] = hex[(bmi_gyro_id >> 4) & 0x0FU];
                    id_text[10] = hex[bmi_gyro_id & 0x0FU];
                }
                OLED_ShowString(0, 32, id_text);
            } else if (shown == 0U) {
                OLED_ShowString(0, 32, (uint8_t *)"I2C:-- E03");
            } else {
                pos = 4U;
                if (shown > sizeof(bmi_scan_addresses)) {
                    shown = sizeof(bmi_scan_addresses);
                }
                for (i = 0U; i < shown; ++i) {
                    text[pos++] = hex[bmi_scan_addresses[i] >> 4];
                    text[pos++] = hex[bmi_scan_addresses[i] & 0x0FU];
                    if (i + 1U < shown) {
                        text[pos++] = ' ';
                    }
                }
                text[pos] = '\0';
                OLED_ShowString(0, 32, text);
            }
        }
        }  /* end of bmi_ok/bmi_attempted static block */

        /* 第4行：目标圈数 */
        OLED_ShowBluetoothReceive();

        if (oled_debug_page != 0U) {
            OLED_DrawDebugPage();
        }

        {
            static uint32_t last_oled_tick = 0U;
            uint32_t now = sys_10ms_tick;
            if ((uint32_t)(now - last_oled_tick) >= 10U) {
                last_oled_tick = now;
                OLED_Refresh_Gram();
            }
        }
    }
}

/* 10ms定时中断 */
void TIMG0_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_LOAD) {

        sys_10ms_tick++;

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
        if (bkeys[1].double_flag == 1) {
            oled_debug_page ^= 1U;
            bkeys[1].double_flag = 0;
        }
    }
}

/* Keep the ISR short: empty the hardware FIFO into a software queue.  Frame
 * parsing stays in the foreground, so it cannot interfere with control ISR. */
void UART1_IRQHandler(void)
{
    uint8_t next_write_index;

    ++bluetooth_uart_irq_count;
    (void)DL_UART_getPendingInterrupt(UART_1_INST);

    while (DL_UART_Main_isRXFIFOEmpty(UART_1_INST) == false) {
        next_write_index =
            (bluetooth_rx_write_index + 1U) & BLUETOOTH_RX_QUEUE_MASK;

        if (next_write_index != bluetooth_rx_read_index) {
            bluetooth_rx_queue[bluetooth_rx_write_index] =
                DL_UART_Main_receiveData(UART_1_INST);
            bluetooth_rx_write_index = next_write_index;
        } else {
            (void)DL_UART_Main_receiveData(UART_1_INST);
            ++bluetooth_rx_overflow_count;
        }
        ++bluetooth_uart_byte_count;
    }
}
