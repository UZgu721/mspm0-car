#include "icm42688.h"
#include "board.h"

/* BMI088 register map and settings, taken from the supplied Bosch BMI088
 * example: accel +/-6 g, gyro +/-1000 dps, both at 100 Hz. */
#define BMI088_REG_CHIP_ID             0x00U

#define BMI088_ACCEL_CHIP_ID_BMI085    0x1AU
#define BMI088_ACCEL_CHIP_ID_BMI088    0x1EU
#define BMI088_GYRO_CHIP_ID            0x0FU

#define BMI088_ACCEL_X_LSB             0x12U
#define BMI088_ACCEL_CONF              0x40U
#define BMI088_ACCEL_RANGE             0x41U
#define BMI088_ACCEL_PWR_CONF          0x7CU
#define BMI088_ACCEL_PWR_CTRL          0x7DU
#define BMI088_ACCEL_SOFTRESET         0x7EU

#define BMI088_GYRO_X_LSB              0x02U
#define BMI088_GYRO_RANGE              0x0FU
#define BMI088_GYRO_BANDWIDTH          0x10U
#define BMI088_GYRO_LPM1               0x11U
#define BMI088_GYRO_SOFTRESET          0x14U

#define BMI088_SOFTRESET_CMD           0xB6U
#define BMI088_ACCEL_CONF_100HZ_NORMAL 0xA8U
#define BMI088_ACCEL_RANGE_6G          0x01U
#define BMI088_ACCEL_POWER_ENABLE      0x04U
#define BMI088_ACCEL_POWER_ACTIVE      0x00U
#define BMI088_GYRO_RANGE_1000DPS      0x01U
#define BMI088_GYRO_BW_32_100HZ        0x07U
#define BMI088_GYRO_POWER_NORMAL       0x00U

#define BMI088_ACCEL_MG_PER_LSB        (6000.0f / 32768.0f)
#define BMI088_GYRO_DPS_PER_LSB        (1000.0f / 32768.0f)
#define BMI088_I2C_TIMEOUT             100000UL
/* BUSCLK is 40 MHz. (1 + 39) * (6 + 4) / 40 MHz = 10 us = 100 kHz. */
#define BMI088_I2C_TPR_100KHZ          39U
#define BMI088_I2C_START_SETTLE_US     2U
#define BMI088_SOFT_I2C_HALF_US         5U
#define BMI088_WRITE_RETRY_COUNT         3U

static uint8_t bmi088_accel_addr;
static uint8_t bmi088_gyro_addr;
static uint8_t bmi088_last_error;
static uint8_t bmi088_init_stage;
static uint32_t bmi088_gyro_read_time_us;

/* SysTick is a free-running 24-bit down-counter clocked at SysTickFre.
 * The measured interval is much shorter than one counter period. */
static uint32_t BMI088_TicksToUs(uint32_t start_tick, uint32_t end_tick)
{
    uint32_t elapsed_ticks;

    if (start_tick >= end_tick) {
        elapsed_ticks = start_tick - end_tick;
    } else {
        elapsed_ticks = start_tick + (SysTickMAX_COUNT + 1U) - end_tick;
    }
    return (elapsed_ticks + (SysTickFre / 2000000U)) /
           (SysTickFre / 1000000U);
}

static uint8_t I2C_HasError(void)
{
    if ((DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
        bmi088_last_error = BMI088_ERROR_I2C_NACK;
        return 1U;
    }
    return 0U;
}

static uint8_t I2C_WaitIdle(void)
{
    uint32_t timeout = BMI088_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (--timeout == 0U) {
            bmi088_last_error = BMI088_ERROR_IDLE_TIMEOUT;
            return 0U;
        }
    }
    return 1U;
}

/* A register-address phase intentionally holds the bus for a repeated START,
 * so it must not wait for BUSY_BUS or IDLE. */
static uint8_t I2C_WaitTransferNoStop(void)
{
    uint32_t timeout = BMI088_I2C_TIMEOUT;

    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        if (I2C_HasError() != 0U) {
            return 0U;
        }
        if (--timeout == 0U) {
            bmi088_last_error = BMI088_ERROR_TRANSFER_TIMEOUT;
            return 0U;
        }
    }

    return (I2C_HasError() == 0U) ? 1U : 0U;
}

static uint8_t I2C_WaitTransferComplete(void)
{
    uint32_t timeout;

    if (I2C_WaitTransferNoStop() == 0U) {
        return 0U;
    }

    timeout = BMI088_I2C_TIMEOUT;
    while ((DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if (I2C_HasError() != 0U) {
            return 0U;
        }
        if (--timeout == 0U) {
            bmi088_last_error = BMI088_ERROR_TRANSFER_TIMEOUT;
            return 0U;
        }
    }
    return I2C_WaitIdle();
}

/* A no-STOP register-address phase keeps the bus busy by design. Wait for
 * the controller's TX-complete event before issuing the repeated START. */
static uint8_t I2C_WaitTxDone(void)
{
    uint32_t timeout = BMI088_I2C_TIMEOUT;

    while (DL_I2C_getRawInterruptStatus(I2C_0_INST,
            DL_I2C_INTERRUPT_CONTROLLER_TX_DONE) == 0U) {
        if (I2C_HasError() != 0U) {
            return 0U;
        }
        if (--timeout == 0U) {
            bmi088_last_error = BMI088_ERROR_TRANSFER_TIMEOUT;
            return 0U;
        }
    }
    DL_I2C_clearInterruptStatus(I2C_0_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    return 1U;
}

static uint8_t I2C_WaitRxDone(void)
{
    uint32_t timeout = BMI088_I2C_TIMEOUT;

    while (DL_I2C_getRawInterruptStatus(I2C_0_INST,
            DL_I2C_INTERRUPT_CONTROLLER_RX_DONE) == 0U) {
        if (I2C_HasError() != 0U) {
            return 0U;
        }
        if (--timeout == 0U) {
            bmi088_last_error = BMI088_ERROR_TRANSFER_TIMEOUT;
            return 0U;
        }
    }
    DL_I2C_clearInterruptStatus(I2C_0_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    return I2C_WaitIdle();
}

/* Temporarily clocks SCL nine times to release a slave that holds SDA low. */
static void I2C_BusRecover(void)
{
    uint8_t i;

    /* A normal idle bus must not be clocked unnecessarily. */
    if ((DL_GPIO_readPins(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN) != 0U) &&
        (DL_GPIO_readPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN) != 0U)) {
        return;
    }

    DL_GPIO_initDigitalOutput(GPIO_I2C_0_IOMUX_SCL);
    DL_GPIO_enableOutput(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    DL_GPIO_setPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    for (i = 0U; i < 9U; ++i) {
        DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_us(5U);
        DL_GPIO_setPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_us(5U);
    }

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SCL,
        GPIO_I2C_0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SDA,
        GPIO_I2C_0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_disableOutput(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SCL);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SDA);
}

static void I2C_ConfigureController(void)
{
    DL_I2C_disableController(I2C_0_INST);
    DL_I2C_resetControllerTransfer(I2C_0_INST);
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_setTimerPeriod(I2C_0_INST, BMI088_I2C_TPR_100KHZ);
    DL_I2C_setControllerTXFIFOThreshold(I2C_0_INST, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setControllerRXFIFOThreshold(I2C_0_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_0_INST);
    DL_I2C_enableController(I2C_0_INST);
}

/* BMI088 uses standard-mode I2C. GPIO output-enable is used as an open-drain
 * driver: drive only low, and release the pin for the external pull-up. This
 * avoids the controller repeated-START state-machine issue seen on this board. */
static void SoftI2C_SdaLow(void)
{
    DL_GPIO_clearPins(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN);
}

static void SoftI2C_SdaRelease(void)
{
    DL_GPIO_disableOutput(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN);
}

static void SoftI2C_SclLow(void)
{
    DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
}

static void SoftI2C_SclRelease(void)
{
    DL_GPIO_disableOutput(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
}

static uint8_t SoftI2C_WaitSclHigh(void)
{
    uint32_t timeout = 1000U;

    while (DL_GPIO_readPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN) == 0U) {
        if (--timeout == 0U) {
            bmi088_last_error = BMI088_ERROR_TRANSFER_TIMEOUT;
            return 0U;
        }
        delay_us(1U);
    }
    return 1U;
}

static void SoftI2C_Start(void)
{
    SoftI2C_SdaRelease();
    SoftI2C_SclRelease();
    (void)SoftI2C_WaitSclHigh();
    delay_us(BMI088_SOFT_I2C_HALF_US);
    SoftI2C_SdaLow();
    delay_us(BMI088_SOFT_I2C_HALF_US);
    SoftI2C_SclLow();
    delay_us(BMI088_SOFT_I2C_HALF_US);
}

static void SoftI2C_Stop(void)
{
    SoftI2C_SdaLow();
    delay_us(BMI088_SOFT_I2C_HALF_US);
    SoftI2C_SclRelease();
    (void)SoftI2C_WaitSclHigh();
    delay_us(BMI088_SOFT_I2C_HALF_US);
    SoftI2C_SdaRelease();
    delay_us(BMI088_SOFT_I2C_HALF_US);
}

static uint8_t SoftI2C_WriteByte(uint8_t value)
{
    uint8_t bit;
    uint8_t ack;

    for (bit = 0x80U; bit != 0U; bit >>= 1U) {
        if ((value & bit) != 0U) {
            SoftI2C_SdaRelease();
        } else {
            SoftI2C_SdaLow();
        }
        delay_us(BMI088_SOFT_I2C_HALF_US);
        SoftI2C_SclRelease();
        if (SoftI2C_WaitSclHigh() == 0U) {
            return 0U;
        }
        delay_us(BMI088_SOFT_I2C_HALF_US);
        SoftI2C_SclLow();
    }

    SoftI2C_SdaRelease();
    delay_us(BMI088_SOFT_I2C_HALF_US);
    SoftI2C_SclRelease();
    if (SoftI2C_WaitSclHigh() == 0U) {
        return 0U;
    }
    delay_us(BMI088_SOFT_I2C_HALF_US);
    ack = (DL_GPIO_readPins(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN) == 0U) ? 1U : 0U;
    SoftI2C_SclLow();
    delay_us(BMI088_SOFT_I2C_HALF_US);
    if (ack == 0U) {
        bmi088_last_error = BMI088_ERROR_I2C_NACK;
    }
    return ack;
}

static uint8_t SoftI2C_ReadByte(uint8_t send_ack)
{
    uint8_t bit;
    uint8_t value = 0U;

    SoftI2C_SdaRelease();
    for (bit = 0x80U; bit != 0U; bit >>= 1U) {
        delay_us(BMI088_SOFT_I2C_HALF_US);
        SoftI2C_SclRelease();
        (void)SoftI2C_WaitSclHigh();
        delay_us(BMI088_SOFT_I2C_HALF_US);
        if (DL_GPIO_readPins(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN) != 0U) {
            value |= bit;
        }
        SoftI2C_SclLow();
    }

    if (send_ack != 0U) {
        SoftI2C_SdaLow();
    } else {
        SoftI2C_SdaRelease();
    }
    delay_us(BMI088_SOFT_I2C_HALF_US);
    SoftI2C_SclRelease();
    (void)SoftI2C_WaitSclHigh();
    delay_us(BMI088_SOFT_I2C_HALF_US);
    SoftI2C_SclLow();
    SoftI2C_SdaRelease();
    return value;
}

static void SoftI2C_Init(void)
{
    DL_I2C_disableController(I2C_0_INST);
    /* Keep GPIO input buffers enabled while output-enable is toggled below.
     * Otherwise released SDA cannot sample a slave ACK. */
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_0_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_0_IOMUX_SCL,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    SoftI2C_SdaRelease();
    SoftI2C_SclRelease();
    delay_us(BMI088_SOFT_I2C_HALF_US);
}

static uint8_t BMI088_WriteReg(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < BMI088_WRITE_RETRY_COUNT; ++attempt) {
        SoftI2C_Start();
        if ((SoftI2C_WriteByte((uint8_t)(address << 1U)) != 0U) &&
            (SoftI2C_WriteByte(reg) != 0U) &&
            (SoftI2C_WriteByte(value) != 0U)) {
            SoftI2C_Stop();
            bmi088_last_error = BMI088_ERROR_NONE;
            return 1U;
        }
        SoftI2C_Stop();
        /* Give a device that just left reset, or a stretched bus, a clean
         * STOP-to-START interval before retrying the whole transaction. */
        delay_ms(2U);
        SoftI2C_Init();
    }
    return 0U;
}

static uint8_t BMI088_ReadRegs(uint8_t address, uint8_t reg, uint8_t *buf,
                               uint8_t length)
{
    uint8_t i;

    if ((buf == 0) || (length == 0U)) {
        return 0U;
    }

    SoftI2C_Start();
    if ((SoftI2C_WriteByte((uint8_t)(address << 1U)) == 0U) ||
        (SoftI2C_WriteByte(reg) == 0U)) {
        SoftI2C_Stop();
        return 0U;
    }
    SoftI2C_Start();
    if (SoftI2C_WriteByte((uint8_t)((address << 1U) | 0x01U)) == 0U) {
        SoftI2C_Stop();
        return 0U;
    }

    for (i = 0U; i < length; ++i) {
        buf[i] = SoftI2C_ReadByte((i + 1U < length) ? 1U : 0U);
    }
    SoftI2C_Stop();
    bmi088_last_error = BMI088_ERROR_NONE;
    return 1U;
}

/* Address scanner helper. The received byte is intentionally discarded: an
 * ACK is all that is needed to report an I2C target address. */
static uint8_t I2C_ProbeAddress(uint8_t address)
{
    SoftI2C_Start();
    if (SoftI2C_WriteByte((uint8_t)((address << 1U) | 0x01U)) == 0U) {
        SoftI2C_Stop();
        return 0U;
    }
    (void)SoftI2C_ReadByte(0U);
    SoftI2C_Stop();
    bmi088_last_error = BMI088_ERROR_NONE;
    return 1U;
}

static uint8_t BMI088_DetectAddress(uint8_t address,
                                    uint8_t expected_id_a, uint8_t expected_id_b,
                                    uint8_t *detected_address)
{
    uint8_t id = 0U;

    /* BMI088 specifies a write of register 0x00 followed by a repeated
     * START for every register read, including CHIP_ID. */
    if (BMI088_ReadRegs(address, BMI088_REG_CHIP_ID, &id, 1U) != 0U &&
        (id == expected_id_a || id == expected_id_b)) {
        *detected_address = address;
        return 1U;
    }
    return 0U;
}

uint8_t BMI088_Init(void)
{
    bmi088_last_error = BMI088_ERROR_NONE;
    bmi088_init_stage = 0U;
    SoftI2C_Init();

    if (BMI088_DetectAddress(BMI088_ACCEL_ADDR,
            BMI088_ACCEL_CHIP_ID_BMI085, BMI088_ACCEL_CHIP_ID_BMI088,
            &bmi088_accel_addr) == 0U) {
        if (bmi088_last_error == BMI088_ERROR_NONE) {
            bmi088_last_error = BMI088_ERROR_ACCEL_NOT_FOUND;
        }
        return 0U;
    }
    if (BMI088_DetectAddress(BMI088_GYRO_ADDR,
            BMI088_GYRO_CHIP_ID, BMI088_GYRO_CHIP_ID, &bmi088_gyro_addr) == 0U) {
        if (bmi088_last_error == BMI088_ERROR_NONE) {
            bmi088_last_error = BMI088_ERROR_GYRO_NOT_FOUND;
        }
        return 0U;
    }

    /* The board already holds reset/startup for 5 s before calling this
     * driver, so both BMI088 dies are in their documented power-on state.
     * On this module the gyro NACKs its 0x14 soft-reset write immediately
     * after the accel 0x7E reset, even though both CHIP_ID reads are valid.
     * Do not issue those redundant independent soft resets; configure the
     * verified power-on state directly instead. */
    delay_ms(2U);

    /* Bosch BMI088 sequence: activate accel PWR_CONF first, then enable
     * PWR_CTRL after 5 ms.  Reversing these two writes can leave the accel
     * suspended, so all later XYZ reads fail despite valid CHIP_ID values. */
    bmi088_init_stage = 3U;
    if (BMI088_WriteReg(bmi088_accel_addr, BMI088_ACCEL_PWR_CONF,
            BMI088_ACCEL_POWER_ACTIVE) == 0U) {
        bmi088_last_error = BMI088_ERROR_CONFIGURATION;
        return 0U;
    }
    delay_ms(5U);
    bmi088_init_stage = 4U;
    if (BMI088_WriteReg(bmi088_accel_addr, BMI088_ACCEL_PWR_CTRL,
            BMI088_ACCEL_POWER_ENABLE) == 0U) {
        bmi088_last_error = BMI088_ERROR_CONFIGURATION;
        return 0U;
    }
    delay_ms(5U);

    bmi088_init_stage = 5U;
    if (BMI088_WriteReg(bmi088_accel_addr, BMI088_ACCEL_CONF,
            BMI088_ACCEL_CONF_100HZ_NORMAL) == 0U ||
        BMI088_WriteReg(bmi088_accel_addr, BMI088_ACCEL_RANGE,
            BMI088_ACCEL_RANGE_6G) == 0U) {
        bmi088_last_error = BMI088_ERROR_CONFIGURATION;
        return 0U;
    }

    /* The gyro needs a full 30 ms after leaving suspend mode before its
     * bandwidth/range registers and data outputs are used. */
    bmi088_init_stage = 6U;
    if (BMI088_WriteReg(bmi088_gyro_addr, BMI088_GYRO_LPM1,
            BMI088_GYRO_POWER_NORMAL) == 0U) {
        bmi088_last_error = BMI088_ERROR_CONFIGURATION;
        return 0U;
    }
    delay_ms(30U);
    bmi088_init_stage = 7U;
    if (BMI088_WriteReg(bmi088_gyro_addr, BMI088_GYRO_BANDWIDTH,
            BMI088_GYRO_BW_32_100HZ) == 0U ||
        BMI088_WriteReg(bmi088_gyro_addr, BMI088_GYRO_RANGE,
            BMI088_GYRO_RANGE_1000DPS) == 0U) {
        bmi088_last_error = BMI088_ERROR_CONFIGURATION;
        return 0U;
    }

    delay_ms(50U);
    bmi088_init_stage = 0U;
    return 1U;
}

uint8_t BMI088_GetLastError(void)
{
    return bmi088_last_error;
}

uint8_t BMI088_GetInitStage(void)
{
    return bmi088_init_stage;
}

uint32_t BMI088_GetGyroReadTimeUs(void)
{
    return bmi088_gyro_read_time_us;
}

uint8_t BMI088_ReadRawIds(uint8_t *accel_id, uint8_t *gyro_id)
{
    uint8_t result = 0U;

    if ((accel_id == 0) || (gyro_id == 0)) {
        return 0U;
    }

    /* The scan has already established that the addresses ACK.  These reads
     * deliberately use the BMI088-required register-write/repeated-START
     * transaction so the OLED can expose the actual ID bytes. */
    SoftI2C_Init();
    if (BMI088_ReadRegs(BMI088_ACCEL_ADDR, BMI088_REG_CHIP_ID,
                        accel_id, 1U) != 0U) {
        result |= 0x01U;
    }
    if (BMI088_ReadRegs(BMI088_GYRO_ADDR, BMI088_REG_CHIP_ID,
                        gyro_id, 1U) != 0U) {
        result |= 0x02U;
    }

    return result;
}

uint8_t BMI088_ScanI2C(uint8_t *addresses, uint8_t max_addresses)
{
    uint8_t address;
    uint8_t count = 0U;

    if ((addresses == 0) || (max_addresses == 0U)) {
        return 0U;
    }

    for (address = 0x08U; address < 0x78U; ++address) {
        if (I2C_ProbeAddress(address) != 0U) {
            if (count < max_addresses) {
                addresses[count] = address;
            }
            ++count;
        }
    }

    if (count == 0U) {
        bmi088_last_error = BMI088_ERROR_I2C_NACK;
    }
    return count;
}

uint8_t BMI088_ReadSample(BMI088_Sample_t *sample)
{
    uint8_t accel[6];
    uint8_t gyro[6];
    uint32_t gyro_read_start;
    uint32_t gyro_read_end;
    int16_t raw;

    if (sample == 0 ||
        BMI088_ReadRegs(bmi088_accel_addr, BMI088_ACCEL_X_LSB, accel, sizeof(accel)) == 0U) {
        return 0U;
    }

    /* Only this six-byte gyro I2C transfer is timed.  The displayed value
     * intentionally excludes accel access, conversion, AHRS and OLED work. */
    gyro_read_start = Systick_getTick();
    if (BMI088_ReadRegs(bmi088_gyro_addr, BMI088_GYRO_X_LSB, gyro,
                        sizeof(gyro)) == 0U) {
        gyro_read_end = Systick_getTick();
        bmi088_gyro_read_time_us = BMI088_TicksToUs(gyro_read_start,
                                                     gyro_read_end);
        return 0U;
    }
    gyro_read_end = Systick_getTick();
    bmi088_gyro_read_time_us = BMI088_TicksToUs(gyro_read_start,
                                                 gyro_read_end);

    raw = (int16_t)((uint16_t)accel[1] << 8 | accel[0]);
    sample->ax = (float)raw * BMI088_ACCEL_MG_PER_LSB;
    raw = (int16_t)((uint16_t)accel[3] << 8 | accel[2]);
    sample->ay = (float)raw * BMI088_ACCEL_MG_PER_LSB;
    raw = (int16_t)((uint16_t)accel[5] << 8 | accel[4]);
    sample->az = (float)raw * BMI088_ACCEL_MG_PER_LSB;

    raw = (int16_t)((uint16_t)gyro[1] << 8 | gyro[0]);
    sample->gx = (float)raw * BMI088_GYRO_DPS_PER_LSB;
    raw = (int16_t)((uint16_t)gyro[3] << 8 | gyro[2]);
    sample->gy = (float)raw * BMI088_GYRO_DPS_PER_LSB;
    raw = (int16_t)((uint16_t)gyro[5] << 8 | gyro[4]);
    sample->gz = (float)raw * BMI088_GYRO_DPS_PER_LSB;
    return 1U;
}
