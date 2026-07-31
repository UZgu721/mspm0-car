#include "huidu.h"
#include "pid.h"
#include "motor.h"

int huidu_Speed = 375;
extern PID_t motorA;
extern PID_t motorB;
extern uint8_t huidu_pid_flag;
extern int32_t vel_left, vel_right;
float last_huidu;

/* G0..G7 are physically arranged from left to right. The gray modules
 * output high on white and low on black; the control mask uses 1 for black. */
#define GRAY_INPUT_MASK (DL_GPIO_PIN_12 | DL_GPIO_PIN_8  | DL_GPIO_PIN_9  | \
                         DL_GPIO_PIN_27 | DL_GPIO_PIN_24 | DL_GPIO_PIN_7  | \
                         DL_GPIO_PIN_22 | DL_GPIO_PIN_15)

PID_t HuiDu_Turn = {
    .Target   = 0,
    .Actual   = 0,
    .Kp       = 0.9,
    .Ki       = 0.0,
    .Kd       = 0.35,
    .Alpha    = 0.3,
    .ErrInt   = 200,
    .Outmax   = 800,
    .Outmin   = -800,
    .PID_MODE = POSITION_PID
};

uint8_t Read_HuiDu(void)
{
    uint32_t pins = DL_GPIO_readPins(GPIOA, GRAY_INPUT_MASK);
    uint8_t black_mask = 0U;

    if ((pins & DL_GPIO_PIN_12) == 0U) black_mask |= 0x01U;
    if ((pins & DL_GPIO_PIN_8)  == 0U) black_mask |= 0x02U;
    if ((pins & DL_GPIO_PIN_9)  == 0U) black_mask |= 0x04U;
    if ((pins & DL_GPIO_PIN_27) == 0U) black_mask |= 0x08U;
    if ((pins & DL_GPIO_PIN_24) == 0U) black_mask |= 0x10U;
    if ((pins & DL_GPIO_PIN_7)  == 0U) black_mask |= 0x20U;
    if ((pins & DL_GPIO_PIN_22) == 0U) black_mask |= 0x40U;
    if ((pins & DL_GPIO_PIN_15) == 0U) black_mask |= 0x80U;

    return black_mask;
}

int Get_HuiDu_Error(void)
{
    uint8_t sensor = Read_HuiDu();
    float weight = 0.0f;
    int count = 0;

    if ((sensor & 0x01U) != 0U) { weight += huidu_Speed * -0.9f; count++; }
    if ((sensor & 0x02U) != 0U) { weight += huidu_Speed * -0.6f; count++; }
    if ((sensor & 0x04U) != 0U) { weight += huidu_Speed * -0.3f; count++; }
    if ((sensor & 0x08U) != 0U) { weight += huidu_Speed * -0.1f; count++; }
    if ((sensor & 0x10U) != 0U) { weight += huidu_Speed *  0.1f; count++; }
    if ((sensor & 0x20U) != 0U) { weight += huidu_Speed *  0.3f; count++; }
    if ((sensor & 0x40U) != 0U) { weight += huidu_Speed *  0.6f; count++; }
    if ((sensor & 0x80U) != 0U) { weight += huidu_Speed *  0.9f; count++; }

    if (count == 0) {
        return 0;
    }
    return (int)(weight / count);
}

/* Normal line-following only. The former straight-run/right-angle-turn
 * state machine, turn counting and lap stopping have been removed. */
void HuiDu_PID(void)
{
    static uint8_t speed_pid_inited = 0U;
    float steer;

    if (huidu_pid_flag == 0U) {
        Motor_SetSpeed(1, 0);
        Motor_SetSpeed(2, 0);
        return;
    }

    /* G4 through G7 are PA24, PA7, PA22 and PA15. All black is the finish
     * marker: stop immediately and let the encoder-based timer freeze. */
    if ((Read_HuiDu() & 0xF0U) == 0xF0U) {
        huidu_pid_flag = 0U;
        Motor_SetSpeed(1, 0);
        Motor_SetSpeed(2, 0);
        return;
    }

    if (speed_pid_inited == 0U) {
        motorA.Out = huidu_Speed;
        motorB.Out = huidu_Speed;
        motorA.Iout = 0;
        motorB.Iout = 0;
        motorA.Target = (vel_left > 0) ? vel_left : 80;
        motorB.Target = (vel_right > 0) ? vel_right : 80;
        speed_pid_inited = 1U;
    }

    motorA.Actual = vel_left;
    motorB.Actual = vel_right;
    PID_Cal(&motorA);
    PID_Cal(&motorB);

    if (motorA.Out > huidu_Speed + 500) motorA.Out = huidu_Speed + 500;
    if (motorA.Out < 150) motorA.Out = 150;
    if (motorB.Out > huidu_Speed + 500) motorB.Out = huidu_Speed + 500;
    if (motorB.Out < 150) motorB.Out = 150;

    last_huidu = HuiDu_Turn.Out;
    HuiDu_Turn.Actual = Get_HuiDu_Error();
    HuiDu_Turn.Target = 0;
    PID_Cal(&HuiDu_Turn);

    steer = HuiDu_Turn.Out * 0.8f + last_huidu * 0.2f;
    Motor_SetSpeed(1, motorA.Out - steer);
    Motor_SetSpeed(2, motorB.Out + steer);
}
