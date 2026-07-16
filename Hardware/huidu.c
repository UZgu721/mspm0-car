#include "huidu.h"
#include "pid.h"
#include "motor.h"
#include "KEY.h"
#include "graysensor.h"

int huidu_Speed = 375;          // 巡线基础速度
extern PID_t motorA;
extern PID_t motorB;
uint8_t turn_count  = 0;        // 转弯计数，每转一次+1
uint8_t lap_count   = 0;        // 圈数，4次转弯=1圈
uint8_t target_lap  = 0;
extern uint8_t huidu_pid_flag;
extern int32_t vel_left, vel_right;
float last_huidu;

/* 灰度传感器实例（全局，供主循环和ISR共享） */
No_MCU_Sensor huidu_sensor;

/* 转向 PID 控制器 */
PID_t HuiDu_Turn = {
    .Target   = 0,
    .Actual   = 0,
    .Kp       = 0.3,           /* 比例减半，防过调冲出 */
    .Ki       = 0.0,
    .Kd       = 0.15,          /* 阻尼匹配减半 */
    .Alpha    = 0.5,
    .ErrInt   = 200,
    .Outmax   = 500,           /* 最大差速减半 */
    .Outmin   = -500,
    .PID_MODE = POSITION_PID
};

/**
 * @brief 灰度传感器初始化（先用默认值，校准会覆盖）
 *        必须在 SYSCFG_DL_init() 后调用
 */
void Huidu_Sensor_Init(void)
{
    /* 默认校准值：白=2000, 黑=800，校准完成后自动覆盖 */
    unsigned short white_def[8] = {2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
    unsigned short black_def[8] = {800, 800, 800, 800, 800, 800, 800, 800};
    No_MCU_Ganv_Sensor_Init(&huidu_sensor, white_def, black_def);
}

/**
 * @brief 开机自动校准：白底上采样，百分比计算阈值
 *        将小车放在白色背景上，上电自动完成
 *        每次调用采集一轮8路ADC，50次后自动切换为百分比阈值
 * @return 1=校准完成
 */
uint8_t Huidu_AutoCalibrate(void)
{
    static uint16_t sample_cnt = 0;
    static uint32_t accum[8] = {0};
    static uint8_t  done = 0;

    if (done) return 1;

    /* 采集一次传感器数据 */
    No_Mcu_Ganv_Sensor_Task_Without_tick(&huidu_sensor);

    /* 累加 Analog_value[0..7] */
    for (int i = 0; i < 8; i++)
        accum[i] += huidu_sensor.Analog_value[i];

    sample_cnt++;
    if (sample_cnt < 50) return 0;   /* 还没采够 */

    /* 50次采完 → 计算均值 + 百分比阈值 */
    unsigned short white_cal[8], black_cal[8];
    for (int i = 0; i < 8; i++) {
        unsigned short avg = (unsigned short)(accum[i] / 50);
        if (avg < 50) avg = 2000;     /* 防止悬空时阈值过低 */
        white_cal[i] = avg;
        black_cal[i] = avg * 25 / 100; /* 黑=白底25%，提高灵敏度 */
    }

    /* 用动态值初始化传感器 */
    No_MCU_Ganv_Sensor_Init(&huidu_sensor, white_cal, black_cal);

    done = 1;
    return 1;
}

/**
 * @brief 灰度传感器主任务（在 while(1) 中持续调用）
 *        采集8路ADC值 → 二值化 → 归一化
 */
void Huidu_Sensor_Task(void)
{
    No_Mcu_Ganv_Sensor_Task_Without_tick(&huidu_sensor);
}

/**
 * @brief 读取8路灰度数字量
 *        bit0~7 = 左1~右8, 1=黑线
 */
uint8_t Read_HuiDu(void)
{
    return Get_Digtal_For_User(&huidu_sensor);
}

/**
 * @brief 8路加权位置误差（左偏为负，右偏为正）
 */
int Get_HuiDu_Error(void)
{
    uint8_t sensor = Read_HuiDu();
    float weight = 0;
    int count = 0;

    if (sensor & 0x01) { weight += huidu_Speed * -0.9f; count++; }
    if (sensor & 0x02) { weight += huidu_Speed * -0.6f; count++; }
    if (sensor & 0x04) { weight += huidu_Speed * -0.3f; count++; }
    if (sensor & 0x08) { weight += huidu_Speed * -0.1f; count++; }
    if (sensor & 0x10) { weight += huidu_Speed *  0.1f; count++; }
    if (sensor & 0x20) { weight += huidu_Speed *  0.3f; count++; }
    if (sensor & 0x40) { weight += huidu_Speed *  0.6f; count++; }
    if (sensor & 0x80) { weight += huidu_Speed *  0.9f; count++; }

    if (count == 0) return 0;
    return (int)(weight / count);
}

/**
 * @brief 巡线主状态机（每10ms由TIMG0中断调用）
 */
void HuiDu_PID(void)
{
    static uint16_t time_1s_go   = 0;
    static uint16_t time_1s_turn = 0;
    static uint8_t  turn_flag     = 0;
    static uint8_t  time_20ms_0   = 0;
    static uint8_t  back_det_flag = 0;

    if (huidu_pid_flag == 0) {
        Motor_SetSpeed(1, 0);
        Motor_SetSpeed(2, 0);
        return;
    }

    /* ── 状态0: 直行巡线 + 路口检测 ── */
    if (turn_flag == 0) {
            uint8_t s = Read_HuiDu() & 0x07;
            uint8_t cnt = (s & 1) + ((s >> 1) & 1) + ((s >> 2) & 1);
            if (cnt >= 2) {
                turn_flag   = 1;       /* 左三路中任意两路黑，立即触发转弯 */
               time_1s_go  = 0;
                time_1s_turn = 0;
            }

            /* ── 速度PID（编码器反馈，闭环稳速，限幅防猛冲） ── */
            {
                static uint8_t speed_pid_inited = 0;
                if (!speed_pid_inited) {
                    motorA.Out    = huidu_Speed;
                    motorB.Out    = huidu_Speed;
                    motorA.Iout   = 0;
                    motorB.Iout   = 0;
                    /* 首帧用当前编码器值作为速度基准 */
                    motorA.Target = (vel_left  > 0) ? vel_left  : 80;
                    motorB.Target = (vel_right > 0) ? vel_right : 80;
                    speed_pid_inited = 1;
                }
                motorA.Actual = vel_left;
                motorB.Actual = vel_right;
                PID_Cal(&motorA);
                PID_Cal(&motorB);
                /* 安全限幅：Out 不超过 base±300，不低于150 */
                if (motorA.Out > huidu_Speed + 300) motorA.Out = huidu_Speed + 300;
                if (motorA.Out < 150) motorA.Out = 150;
                if (motorB.Out > huidu_Speed + 300) motorB.Out = huidu_Speed + 300;
                if (motorB.Out < 150) motorB.Out = 150;
            }

            /* ── 转向PID（灰度反馈，差速纠偏） ── */
            last_huidu = HuiDu_Turn.Out;
            HuiDu_Turn.Actual = Get_HuiDu_Error();
            HuiDu_Turn.Target = 0;
            PID_Cal(&HuiDu_Turn);

            /* 速度输出 + 转向差速 */
            float steer = HuiDu_Turn.Out * 0.5f + last_huidu * 0.5f;
            Motor_SetSpeed(1, motorA.Out - steer);
            Motor_SetSpeed(2, motorB.Out + steer);

    /* ── 状态1: 转弯 ── */
    } else if (turn_flag == 1) {
        if (time_1s_go <= 90) {
            time_1s_go++;
            Motor_SetSpeed(1, 200);
            Motor_SetSpeed(2, 200);
        } else {
            if (time_1s_turn <= 200) {
                time_1s_turn++;
                Motor_SetSpeed(1, -200);
                Motor_SetSpeed(2,  200);

                if ((Read_HuiDu() & 0xF0) != 0) {
                    back_det_flag = 1;
                    if (time_20ms_0 >= 2) {
                        time_20ms_0 = 0;
                        back_det_flag = 0;
                        if ((Read_HuiDu() & 0xF0) != 0) {
                            time_1s_turn = 0;
                            time_1s_go   = 0;
                            turn_flag    = 0;
                            turn_count++;
                            if (turn_count >= 4) {
                                turn_count = 0;
                                lap_count++;
                                if (target_lap > 0 && lap_count >= target_lap) {
                                    huidu_pid_flag = 0;
                                    Motor_SetSpeed(1, 0);
                                    Motor_SetSpeed(2, 0);
                                }
                            }
                            return;
                        }
                    }
                } else {
                    back_det_flag = 0;
                    time_20ms_0 = 0;
                }
                if (back_det_flag) time_20ms_0++;
            } else {
                time_1s_turn = 0;
                time_1s_go   = 0;
                turn_flag    = 0;
                turn_count++;
                if (turn_count >= 4) {
                    turn_count = 0;
                    lap_count++;
                    if (target_lap > 0 && lap_count >= target_lap) {
                        huidu_pid_flag = 0;
                        Motor_SetSpeed(1, 0);
                        Motor_SetSpeed(2, 0);
                    }
                }
            }
        }
    }
}
