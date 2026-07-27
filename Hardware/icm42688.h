#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/*
 * BMI088 I2C driver interface.
 *
 * The source file retains its historical name (icm42688.c) because CCS has
 * already generated a make rule for that source file.  The implementation and
 * all public symbols below are BMI088-only; no ICM42688 register is used.
 */

/* This board straps SDO1 and SDO2 high. These are the same secondary I2C
 * addresses selected by the supplied Bosch BMI088 I2C example. */
#define BMI088_ACCEL_ADDR              0x19U
#define BMI088_GYRO_ADDR               0x69U

typedef struct {
    float gx;     /* dps */
    float gy;     /* dps */
    float gz;     /* dps */
    float ax;     /* mg  */
    float ay;     /* mg  */
    float az;     /* mg  */
} BMI088_Sample_t;

/* Runtime diagnostic codes for the latest failed I2C/BMI088 operation. */
#define BMI088_ERROR_NONE              0U
#define BMI088_ERROR_IDLE_TIMEOUT      1U
#define BMI088_ERROR_TRANSFER_TIMEOUT  2U
#define BMI088_ERROR_I2C_NACK          3U
#define BMI088_ERROR_ACCEL_NOT_FOUND   4U
#define BMI088_ERROR_GYRO_NOT_FOUND    5U
#define BMI088_ERROR_CONFIGURATION     6U

/* Returns 1 only after both BMI088 chips are detected and configured. */
uint8_t BMI088_Init(void);

/* Reads each chip's contiguous XYZ register block in one I2C transaction. */
uint8_t BMI088_ReadSample(BMI088_Sample_t *sample);

uint8_t BMI088_GetLastError(void);
uint8_t BMI088_GetInitStage(void);
uint32_t BMI088_GetGyroReadTimeUs(void);

/* Reads register 0x00 from both devices without applying configuration.
 * Return bit0: accelerometer CHIP_ID read successfully; bit1: gyroscope. */
uint8_t BMI088_ReadRawIds(uint8_t *accel_id, uint8_t *gyro_id);

/* One-shot diagnostic: scans legal 7-bit I2C addresses and stores each
 * address that acknowledges a single-byte read. */
uint8_t BMI088_ScanI2C(uint8_t *addresses, uint8_t max_addresses);

#endif
