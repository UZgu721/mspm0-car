#include "encoder.h"

volatile int32_t Get_Encoder_countA, Get_Encoder_countB;

/**
 * @brief 编码器初始化：配置为双边沿触发（电平状态机方案）
 *        必须在 SYSCFG_DL_init() 之后调用，覆盖默认的上升沿配置
 */
void Encoder_Init(void)
{
    /* 编码器A: PA25(E1A) + PA26(E1B) → 双边沿检测 */
    DL_GPIO_setUpperPinsPolarity(ENCODERA_PORT,
        DL_GPIO_PIN_25_EDGE_RISE_FALL | DL_GPIO_PIN_26_EDGE_RISE_FALL);

    /* 编码器B: PB20(E2A) + PB24(E2B) → 双边沿检测 */
    DL_GPIO_setUpperPinsPolarity(ENCODERB_PORT,
        DL_GPIO_PIN_20_EDGE_RISE_FALL | DL_GPIO_PIN_24_EDGE_RISE_FALL);
}

/**
 * @brief GROUP1 中断服务函数（GPIOA + GPIOB 共享）
 *
 * 电平状态机正交解码（4倍频）：
 *   - A/B 两路直接设为 EDGE_RISE_FALL，不再动态翻转极性
 *   - 中断触发后读取 A、B 当前电平 → 2-bit 状态
 *   - 与上次状态组成 4-bit 转移码 = (prev << 2) | curr
 *   - 查正交状态转移表判断方向并计数
 *
 * 正交状态转移表（A=bit1, B=bit0）:
 *   CW  (00→01→11→10→00): 0x01, 0x07, 0x0E, 0x08
 *   CCW (00→10→11→01→00): 0x02, 0x0B, 0x0D, 0x04
 *   其他转移码 → 干扰或丢边沿，忽略不计
 */
void GROUP1_IRQHandler(void)
{
    static uint8_t encA_state = 0; /* 编码器A上一次AB电平状态 */
    static uint8_t encB_state = 0; /* 编码器B上一次AB电平状态 */
    uint8_t currA, currB;
    uint8_t trans;
    uint32_t statusA, statusB;

    /* ── 编码器A: GPIOA (PA25=E1A, PA26=E1B) ── */
    statusA = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,
                  ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);

    if (statusA) {
        /* 读当前AB电平: bit1=A(PA25), bit0=B(PA26) */
        currA = 0;
        if (DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN))
            currA |= 0x02;
        if (DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN))
            currA |= 0x01;

        /* 4-bit 转移码 */
        trans = (uint8_t)((encA_state << 2) | currA);

        /* 查表判断方向 */
        if (trans == 0x01 || trans == 0x07 ||
            trans == 0x0E || trans == 0x08) {
            Get_Encoder_countA++;       /* CW 正转 */
        } else if (trans == 0x02 || trans == 0x0B ||
                   trans == 0x0D || trans == 0x04) {
            Get_Encoder_countA--;       /* CCW 反转 */
        }
        /* 其他 → 非法转移，忽略 */

        encA_state = currA;
        DL_GPIO_clearInterruptStatus(ENCODERA_PORT,
            ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    }

    /* ── 编码器B: GPIOB (PB20=E2A, PB24=E2B) ── */
    statusB = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,
                  ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    if (statusB) {
        /* 读当前AB电平: bit1=A(PB20), bit0=B(PB24) */
        currB = 0;
        if (DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN))
            currB |= 0x02;
        if (DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN))
            currB |= 0x01;

        /* 4-bit 转移码 */
        trans = (uint8_t)((encB_state << 2) | currB);

        /* 编码器B方向与A相反（右轮电机对向安装） */
        if (trans == 0x01 || trans == 0x07 ||
            trans == 0x0E || trans == 0x08) {
            Get_Encoder_countB--;       /* CW → 电机反转 */
        } else if (trans == 0x02 || trans == 0x0B ||
                   trans == 0x0D || trans == 0x04) {
            Get_Encoder_countB++;       /* CCW → 电机正转 */
        }

        encB_state = currB;
        DL_GPIO_clearInterruptStatus(ENCODERB_PORT,
            ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
    }
}

/**
 * @brief 读取左轮编码器增量值（读后清零）
 * @return 本次调用与上次调用之间的脉冲增量
 */
int32_t Get_encoder_left()
{
    int32_t i = Get_Encoder_countA;
    Get_Encoder_countA = 0;
    return i;
}

/**
 * @brief 读取右轮编码器增量值（读后清零）
 * @return 本次调用与上次调用之间的脉冲增量
 */
int32_t Get_encoder_right()
{
    int32_t i = Get_Encoder_countB;
    Get_Encoder_countB = 0;
    return i;
}
