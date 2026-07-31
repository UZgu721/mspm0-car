/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */
#include "board.h"
#include "ti_msp_dl_config.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "huidu.h"

uint8_t i = 0;
int32_t vel_left = 0, vel_right = 0;
uint8_t time_40ms = 0;
extern PID_t motorA, motorB;
uint8_t pid_flag = 0, huidu_pid_flag = 0;
volatile uint32_t sys_10ms_tick = 0;

#define RUN_TIMER_STOP_CONFIRM_TICKS 50U

typedef enum {
    RUN_TIMER_IDLE = 0,
    RUN_TIMER_RUNNING,
    RUN_TIMER_STOPPED
} RunTimerState;

static volatile RunTimerState run_timer_state = RUN_TIMER_IDLE;
static volatile uint32_t run_timer_ticks;
static volatile uint32_t run_timer_last_motion_ticks;
static volatile uint8_t run_timer_motion_seen;
static volatile uint8_t run_timer_still_ticks;

/* UART1 receives Bluetooth/serial text on PB7. RX is independent from the
 * control loop and no attitude data is transmitted. */
#define BLUETOOTH_RX_PACKET_LENGTH 18U
#define BLUETOOTH_RX_QUEUE_SIZE    32U
#define BLUETOOTH_RX_QUEUE_MASK    (BLUETOOTH_RX_QUEUE_SIZE - 1U)

static uint8_t bluetooth_rx_packet[BLUETOOTH_RX_PACKET_LENGTH + 1U] = "---";
static uint8_t bluetooth_rx_packet_length;
static uint8_t bluetooth_rx_packet_complete;
static volatile uint8_t bluetooth_rx_queue[BLUETOOTH_RX_QUEUE_SIZE];
static volatile uint8_t bluetooth_rx_write_index;
static volatile uint8_t bluetooth_rx_read_index;
static volatile uint16_t bluetooth_rx_overflow_count;
static volatile uint16_t bluetooth_uart_irq_count;
static volatile uint16_t bluetooth_uart_byte_count;

static uint32_t RunTimer_GetSeconds(void)
{
    return run_timer_ticks / 100U;
}

static void RunTimer_Start(void)
{
    run_timer_state = RUN_TIMER_RUNNING;
    run_timer_ticks = 0U;
    run_timer_last_motion_ticks = 0U;
    run_timer_motion_seen = 0U;
    run_timer_still_ticks = 0U;
}

/* A software stop (finish marker) freezes immediately. If the car becomes
 * physically stuck while still commanded to run, the encoder-based 500 ms
 * no-pulse rule remains as a fallback. */
static void RunTimer_Task(void)
{
    if (run_timer_state != RUN_TIMER_RUNNING) {
        return;
    }

    if (huidu_pid_flag == 0U) {
        run_timer_state = RUN_TIMER_STOPPED;
        return;
    }

    ++run_timer_ticks;

    if ((vel_left != 0) || (vel_right != 0)) {
        run_timer_motion_seen = 1U;
        run_timer_still_ticks = 0U;
        run_timer_last_motion_ticks = run_timer_ticks;
    } else if (run_timer_motion_seen != 0U) {
        if (run_timer_still_ticks < RUN_TIMER_STOP_CONFIRM_TICKS) {
            ++run_timer_still_ticks;
        }
        if (run_timer_still_ticks >= RUN_TIMER_STOP_CONFIRM_TICKS) {
            run_timer_ticks = run_timer_last_motion_ticks;
            run_timer_state = RUN_TIMER_STOPPED;
        }
    }
}

/* PB8 is active-low. Only a release after at least 700 ms starts following. */
static void Key_ServiceLongPress(void)
{
    static uint8_t pressed_ticks = 0U;

    if (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_8) == 0U) {
        if (pressed_ticks < 255U) {
            ++pressed_ticks;
        }
    } else if (pressed_ticks != 0U) {
        if (pressed_ticks > 69U) {
            huidu_pid_flag = 1U;
            if (run_timer_state != RUN_TIMER_RUNNING) {
                RunTimer_Start();
            }
        }
        pressed_ticks = 0U;
    }
}

/* A '<' starts a new framed message; CR/LF terminates ordinary text. */
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

static void Bluetooth_Task(void)
{
    uint8_t byte;

    while (bluetooth_rx_read_index != bluetooth_rx_write_index) {
        byte = bluetooth_rx_queue[bluetooth_rx_read_index];
        bluetooth_rx_read_index =
            (bluetooth_rx_read_index + 1U) & BLUETOOTH_RX_QUEUE_MASK;
        Bluetooth_StoreRawByte(byte);
    }
}

int main(void)
{
    SYSCFG_DL_init();

    Encoder_Init();
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
        Bluetooth_Task();

        memset(OLED_GRAM, 0, 128 * 8 * sizeof(u8));

        OLED_ShowString(0, 0, (uint8_t *)"L:");
        OLED_ShowSignedNum(12, 0, vel_left, 6, 12);
        OLED_ShowString(54, 0, (uint8_t *)"R:");
        OLED_ShowSignedNum(66, 0, vel_right, 6, 12);

        {
            uint8_t g = Read_HuiDu();
            uint8_t buf[12];

            buf[0] = 'G';
            buf[1] = ':';
            for (int index = 0; index < 8; ++index) {
                buf[2 + index] = (g & (1U << index)) ? '1' : '0';
            }
            buf[10] = '\0';
            OLED_ShowString(0, 16, buf);
        }

        OLED_ShowString(0, 32, (uint8_t *)"RUN:");
        OLED_ShowString(30, 32,
                        (uint8_t *)(huidu_pid_flag != 0U ? "ON" : "WAIT"));
        OLED_ShowString(0, 48, (uint8_t *)"T:");
        OLED_ShowNumber(12, 48, RunTimer_GetSeconds(), 5, 12);
        OLED_ShowString(42, 48, (uint8_t *)"s");

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

/* 10 ms timer interrupt */
void TIMG0_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_LOAD) {
        sys_10ms_tick++;
        vel_left = Get_encoder_left();
        vel_right = Get_encoder_right();

        if (huidu_pid_flag == 1U) {
            HuiDu_PID();
        }
        Key_ServiceLongPress();
        RunTimer_Task();
    }
}

/* Keep the ISR short: empty the hardware FIFO into a software queue. */
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
