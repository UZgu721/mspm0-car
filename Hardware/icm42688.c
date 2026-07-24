#include "icm42688.h"
#include "board.h"   /* delay_us, delay_ms */

/* ── 全局变量 ── */
float icm42688_acc_x, icm42688_acc_y, icm42688_acc_z;
float icm42688_gyro_x, icm42688_gyro_y, icm42688_gyro_z;
float gx, gy, gz;
float ax, ay, az;

/* 转换系数 */
static float icm42688_acc_inv  = 16000.0f / 32768.0f;
static float icm42688_gyro_inv = 2000.0f / 32768.0f;

/* 陀螺 Z 轴零偏（启动校准） */
float gz_bias = 0.0f;

/* ── I2C 辅助（参照 MSPM0 官方桥接示例） ── */

/**
 * @brief I2C 总线恢复：SCL发9个时钟释放被锁死的SDA
 */
static void I2C_BusRecover(void)
{
    /* 临时将 SCL(PA1) 改为 GPIO 输出 */
    DL_GPIO_initDigitalOutput(GPIO_I2C_0_IOMUX_SCL);
    for (int i = 0; i < 9; i++) {
        DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_us(5);
        DL_GPIO_setPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_us(5);
    }
    /* 恢复为 I2C 开漏外设功能（与 SYSCFG_DL_GPIO_init 一致） */
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SCL,
        GPIO_I2C_0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SDA,
        GPIO_I2C_0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SCL);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SDA);
}

/**
 * @brief I2C 总线恢复：确保控制器空闲再开始
 */
static void I2C_WaitIdle(void)
{
    uint32_t to = 100000;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)
           && --to);
}

/**
 * @brief 写 ICM42688 寄存器（2字节：reg + data）
 */
static void ICM42688_WriteReg(uint8_t reg, uint8_t data)
{
    uint32_t to;
    uint8_t tx[2] = { reg, data };

    I2C_WaitIdle();
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, tx, 2);
    DL_I2C_startControllerTransfer(I2C_0_INST, ICM42688_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    to = 100000;
    while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY)
           && --to);
    to = 100000;
    while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
           && --to);
    I2C_WaitIdle();
}

/**
 * @brief 读 ICM42688 多寄存器（发reg→repeated START→读len字节）
 */
static void ICM42688_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint32_t to;

    /* Phase 1: 发寄存器地址，不产生 STOP */
    I2C_WaitIdle();
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, ICM42688_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_DISABLE,
        DL_I2C_CONTROLLER_ACK_ENABLE);
    to = 100000;
    while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY)
           && --to);

    /* Phase 2: Repeated START + 读 + STOP */
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, ICM42688_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_ENABLE);
    to = 100000;
    while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY)
           && --to);
    to = 100000;
    while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
           && --to);
    I2C_WaitIdle();

    /* 读取 RX FIFO */
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
}

/* ── 量程/ODR 配置 ── */

void ICM42688_Set_Range(enum icm42688_afs afs, enum icm42688_aodr aodr,
                        enum icm42688_gfs gfs, enum icm42688_godr godr)
{
    ICM42688_WriteReg(ICM42688_ACCEL_CONFIG0, (uint8_t)((afs << 5) | (aodr + 1)));
    ICM42688_WriteReg(ICM42688_GYRO_CONFIG0,  (uint8_t)((gfs << 5) | (godr + 1)));

    switch (afs) {
    case ICM42688_AFS_2G:  icm42688_acc_inv = 2000.0f  / 32768.0f; break;
    case ICM42688_AFS_4G:  icm42688_acc_inv = 4000.0f  / 32768.0f; break;
    case ICM42688_AFS_8G:  icm42688_acc_inv = 8000.0f  / 32768.0f; break;
    case ICM42688_AFS_16G: icm42688_acc_inv = 16000.0f / 32768.0f; break;
    default: break;
    }
    switch (gfs) {
    case ICM42688_GFS_15_625DPS: icm42688_gyro_inv = 15.625f  / 32768.0f; break;
    case ICM42688_GFS_31_25DPS:  icm42688_gyro_inv = 31.25f   / 32768.0f; break;
    case ICM42688_GFS_62_5DPS:   icm42688_gyro_inv = 62.5f    / 32768.0f; break;
    case ICM42688_GFS_125DPS:    icm42688_gyro_inv = 125.0f   / 32768.0f; break;
    case ICM42688_GFS_250DPS:    icm42688_gyro_inv = 250.0f   / 32768.0f; break;
    case ICM42688_GFS_500DPS:    icm42688_gyro_inv = 500.0f   / 32768.0f; break;
    case ICM42688_GFS_1000DPS:   icm42688_gyro_inv = 1000.0f  / 32768.0f; break;
    case ICM42688_GFS_2000DPS:   icm42688_gyro_inv = 2000.0f  / 32768.0f; break;
    default: break;
    }
}

/* ── 初始化 ── */

/**
 * @brief ICM42688 初始化，返回 1=成功 0=失败（不阻塞系统）
 */
uint8_t ICM42688_Init(void)
{
    uint8_t whoami = 0;
    uint8_t retry  = 50;

    /* 总线恢复（释放可能被锁死的 SDA） */
    I2C_BusRecover();

    /* 完善 I2C0 配置（SysConfig 生成的 init 不足） */
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_setTimerPeriod(I2C_0_INST, 7);                /* 400kHz @ 40MHz BUSCLK */
    DL_I2C_setControllerTXFIFOThreshold(I2C_0_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setControllerRXFIFOThreshold(I2C_0_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableController(I2C_0_INST);

    /* WHO_AM_I 检测（500ms 超时） */
    while (retry--) {
        ICM42688_ReadRegs(ICM42688_WHO_AM_I, &whoami, 1);
        if (whoami == ICM42688_WHO_AM_I_VAL) break;
        delay_ms(10);
    }
    if (whoami != ICM42688_WHO_AM_I_VAL) {
        return 0;   /* 未检测到，返回失败 */
    }

    /* 器件复位 */
    ICM42688_WriteReg(ICM42688_PWR_MGMT0, 0x00);
    delay_ms(10);

    /* 配置量程和 ODR */
    ICM42688_Set_Range(ICM42688_AFS_16G, ICM42688_AODR_1000HZ,
                       ICM42688_GFS_2000DPS, ICM42688_GODR_1000HZ);

    /* 使能陀螺仪和加速度计 */
    ICM42688_WriteReg(ICM42688_PWR_MGMT0, 0x0F);
    delay_ms(200);  /* 等传感器稳定 */

    /* Bias calibration is performed once by IMU_InitAndCalibrate(). */
    gz_bias = 0.0f;

    return 1;   /* 初始化成功 */
}

/* ── 数据读取 ── */

void ICM42688_Read_Accel(void)
{
    uint8_t xh, xl, yh, yl, zh, zl;
    /* 逐字节读，避免 I2C burst 自动增量问题 */
    ICM42688_ReadRegs(ICM42688_ACCEL_DATA_X1, &xh, 1);
    ICM42688_ReadRegs(ICM42688_ACCEL_DATA_X0, &xl, 1);
    ICM42688_ReadRegs(ICM42688_ACCEL_DATA_Y1, &yh, 1);
    ICM42688_ReadRegs(ICM42688_ACCEL_DATA_Y0, &yl, 1);
    ICM42688_ReadRegs(ICM42688_ACCEL_DATA_Z1, &zh, 1);
    ICM42688_ReadRegs(ICM42688_ACCEL_DATA_Z0, &zl, 1);
    icm42688_acc_x = icm42688_acc_inv * (int16_t)(((uint16_t)xh << 8) | xl);
    icm42688_acc_y = icm42688_acc_inv * (int16_t)(((uint16_t)yh << 8) | yl);
    icm42688_acc_z = icm42688_acc_inv * (int16_t)(((uint16_t)zh << 8) | zl);
    ax = icm42688_acc_x;
    ay = icm42688_acc_y;
    az = icm42688_acc_z;
}

/**
 * @brief 只读陀螺Z轴（快通道，2次I2C ~0.5ms）
 *        不影响灰度巡线刷新率
 */
void ICM42688_Read_GyroZ(void)
{
    uint8_t zh, zl;
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_Z1, &zh, 1);
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_Z0, &zl, 1);
    icm42688_gyro_z = icm42688_gyro_inv * (int16_t)(((uint16_t)zh << 8) | zl);
    gz = icm42688_gyro_z - gz_bias;
}

void ICM42688_Read_Gyro(void)
{
    uint8_t xh, xl, yh, yl, zh, zl;
    /* 逐字节读，避免 I2C burst 自动增量问题 */
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_X1, &xh, 1);
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_X0, &xl, 1);
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_Y1, &yh, 1);
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_Y0, &yl, 1);
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_Z1, &zh, 1);
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_Z0, &zl, 1);
    icm42688_gyro_x = icm42688_gyro_inv * (int16_t)(((uint16_t)xh << 8) | xl);
    icm42688_gyro_y = icm42688_gyro_inv * (int16_t)(((uint16_t)yh << 8) | yl);
    icm42688_gyro_z = icm42688_gyro_inv * (int16_t)(((uint16_t)zh << 8) | zl);
    gx = icm42688_gyro_x;
    gy = icm42688_gyro_y;
    gz = icm42688_gyro_z - gz_bias;  /* 消零偏 */
}

/* Read each sensor block in one transaction.  Six bytes fits the MSPM0 I2C
 * RX FIFO and avoids combining different sampling instants byte by byte. */
void ICM42688_ReadSample(ICM42688_Sample_t *sample)
{
    uint8_t acc[6];
    uint8_t gyro[6];

    if (sample == 0) {
        return;
    }

    ICM42688_ReadRegs(ICM42688_ACCEL_DATA_X1, acc, 6);
    ICM42688_ReadRegs(ICM42688_GYRO_DATA_X1, gyro, 6);

    sample->ax = icm42688_acc_inv * (float)(int16_t)(((uint16_t)acc[0] << 8) | acc[1]);
    sample->ay = icm42688_acc_inv * (float)(int16_t)(((uint16_t)acc[2] << 8) | acc[3]);
    sample->az = icm42688_acc_inv * (float)(int16_t)(((uint16_t)acc[4] << 8) | acc[5]);
    sample->gx = icm42688_gyro_inv * (float)(int16_t)(((uint16_t)gyro[0] << 8) | gyro[1]);
    sample->gy = icm42688_gyro_inv * (float)(int16_t)(((uint16_t)gyro[2] << 8) | gyro[3]);
    sample->gz = icm42688_gyro_inv * (float)(int16_t)(((uint16_t)gyro[4] << 8) | gyro[5]);

    icm42688_acc_x = ax = sample->ax;
    icm42688_acc_y = ay = sample->ay;
    icm42688_acc_z = az = sample->az;
    icm42688_gyro_x = gx = sample->gx;
    icm42688_gyro_y = gy = sample->gy;
    icm42688_gyro_z = gz = sample->gz;
}
