/**
 * @file MPU_6050.h
 * @brief Header file for the MPU_6050 driver.
 *
 * This file contains the function definitions for the MPU_6050 driver.
 * It interfaces with the MPU-6050 6-DoF Accelerometer and Gyroscope Sensor module, which uses the I2C communication protocol.
 *  - Product Link: https://www.adafruit.com/product/3886
 *  - Datasheet: http://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
 *  - Register Map: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * The MPU-6050 module uses the following SPI configuration:
 *  - SCL Frequency: 1 MHz
 *  - MSB First
 *
 * The following connections must be made:
 *  - MPU-6050 VIN      <-->  MSP432 LaunchPad Pin VCC (3.3V)
 *  - MPU-6050 3Vo      <-->  Not Connected
 *  - MPU-6050 GND      <-->  MSP432 LaunchPad Pin GND
 *  - MPU-6050 SCL      <-->  MSP432 LaunchPad Pin P6.5 (SCL)
 *  - MPU-6050 SDA      <-->  MSP432 LaunchPad Pin P6.4 (SDA)
 *  - MPU-6050 INT      <-->  Not Connected
 *
 * For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#ifndef MPU_6050_H_
#define MPU_6050_H_

#include <stdint.h>
#include "msp.h"
#include "../inc/EUSCI_B1_I2C.h"
#include "../inc/Clock.h"

// Default I2C address for the MPU-6050
#define MPU_6050_ADDRESS                        0x68

// Register Bit Position Values
#define MPU_REG_SET_BIT_0                       0x01
#define MPU_REG_SET_BIT_1                       0x02
#define MPU_REG_SET_BIT_2                       0x04
#define MPU_REG_SET_BIT_3                       0x08
#define MPU_REG_SET_BIT_4                       0x10
#define MPU_REG_SET_BIT_5                       0x20
#define MPU_REG_SET_BIT_6                       0x40
#define MPU_REG_SET_BIT_7                       0x80

// Clock Source Select Values (CLKSEL[2:0], PWR_MGMT_1 Register)
#define CLKSEL_PLL_INTERNAL_8_MHZ               0x00
#define CLKSEL_PLL_X_AXIS_GYRO_REFERENCE        0x01
#define CLKSEL_PLL_Y_AXIS_GYRO_REFERENCE        0x02
#define CLKSEL_PLL_Z_AXIS_GYRO_REFERENCE        0x03
#define CLKSEL_PLL_EXTERNAL_32_768_KHZ          0x04
#define CLKSEL_PLL_EXTERNAL_19_2_MHZ            0x05
#define CLKSEL_PLL_RESERVED                     0x06
#define CLKSEL_STOP_CLK                         0x07

// Digital Low-Pass Filter Bandwidth Select Values (DLPF_CFG[2:0], CONFIG Register)
#define MPU_6050_DLPF_CFG_BANDWIDTH_260_HZ      0x00
#define MPU_6050_DLPF_CFG_BANDWIDTH_184_HZ      0x01
#define MPU_6050_DLPF_CFG_BANDWIDTH_94_HZ       0x02
#define MPU_6050_DLPF_CFG_BANDWIDTH_44_HZ       0x03
#define MPU_6050_DLPF_CFG_BANDWIDTH_21_HZ       0x04
#define MPU_6050_DLPF_CFG_BANDWIDTH_10_HZ       0x05
#define MPU_6050_DLPF_CFG_BANDWIDTH_5_HZ        0x06
#define MPU_6050_DLPF_CFG_RESERVED              0x07

// Accelerometer Range Select Values (AFS_SEL[1:0], ACCEL_CONFIG Register)
#define MPU_6050_ACCEL_RANGE_2_G                (0x00)
#define MPU_6050_ACCEL_RANGE_4_G                (0x01 << 3)
#define MPU_6050_ACCEL_RANGE_8_G                (0x02 << 3)
#define MPU_6050_ACCEL_RANGE_16_G               (0x03 << 3)

// Gyroscope Range Select Values (FS_SEL[1:0], GYRO_CONFIG Register)
#define MPU_6050_GYRO_RANGE_250_DEG             (0x00)
#define MPU_6050_GYRO_RANGE_500_DEG             (0x01 << 3)
#define MPU_6050_GYRO_RANGE_1000_DEG            (0x02 << 3)
#define MPU_6050_GYRO_RANGE_2000_DEG            (0x03 << 3)

// Register Address Definitions
#define MPU_6050_SMPRT_DIV                      0x19
#define MPU_6050_CONFIG                         0x1A
#define MPU_6050_GYRO_CONFIG                    0x1B
#define MPU_6050_ACCEL_CONFIG                   0x1C
#define MPU_6050_SIGNAL_PATH_RESET              0x68
#define MPU_6050_PWR_MGMT_1                     0x6B
#define MPU_6050_WHO_AM_I                       0x75

#define MPU_6050_ACCEL_XOUT_H                   0x3B
#define MPU_6050_ACCEL_XOUT_L                   0x3C
#define MPU_6050_ACCEL_YOUT_H                   0x3D
#define MPU_6050_ACCEL_YOUT_L                   0x3E
#define MPU_6050_ACCEL_ZOUT_H                   0x3F
#define MPU_6050_ACCEL_ZOUT_L                   0x40

#define MPU_6050_GYRO_XOUT_H                    0x43
#define MPU_6050_GYRO_XOUT_L                    0x44
#define MPU_6050_GYRO_YOUT_H                    0x45
#define MPU_6050_GYRO_YOUT_L                    0x46
#define MPU_6050_GYRO_ZOUT_H                    0x47
#define MPU_6050_GYRO_ZOUT_L                    0x48

void MPU_6050_Write_Register(uint8_t register_address, uint8_t register_data);

uint8_t MPU_6050_Read_Register(uint8_t register_address);

void MPU_6050_Init();

void MPU_6050_Reset();

uint8_t MPU_6050_Get_Device_ID();

void MPU_6050_Set_Clock_Source(uint8_t clock_source_select);

uint8_t MPU_6050_Get_Clock_Source();

void MPU_6050_Set_Sample_Rate_Divider(uint8_t sample_rate_divider);

uint8_t MPU_6050_Get_Sample_Rate_Divider();

void MPU_6050_Set_DLPF_Bandwidth(uint8_t dlpf_cfg);

uint8_t MPU_6050_Get_DLPF_Bandwidth();

void MPU_6050_Set_Accelerometer_Range(uint8_t accelerometer_range);

uint8_t MPU_6050_Get_Accelerometer_Range();

int MPU_6050_Get_Accelerometer_Scale(uint8_t accelerometer_range);

uint8_t MPU_6050_Get_Accel_X_High_Byte();

uint8_t MPU_6050_Get_Accel_X_Low_Byte();

uint8_t MPU_6050_Get_Accel_Y_High_Byte();

uint8_t MPU_6050_Get_Accel_Y_Low_Byte();

uint8_t MPU_6050_Get_Accel_Z_High_Byte();

uint8_t MPU_6050_Get_Accel_Z_Low_Byte();

int16_t* MPU_6050_Get_Raw_XYZ_Acceleration();

float* MPU_6050_Get_Adjusted_XYZ_Acceleration();

void MPU_6050_Set_Gyroscope_Range(uint8_t gyroscope_range);

uint8_t MPU_6050_Get_Gyroscope_Range();

float MPU_6050_Get_Gyroscope_Scale(uint8_t gyroscope_range);

uint8_t MPU_6050_Get_Gyro_X_High_Byte();

uint8_t MPU_6050_Get_Gyro_X_Low_Byte();

uint8_t MPU_6050_Get_Gyro_Y_High_Byte();

uint8_t MPU_6050_Get_Gyro_Y_Low_Byte();

uint8_t MPU_6050_Get_Gyro_Z_High_Byte();

uint8_t MPU_6050_Get_Gyro_Z_Low_Byte();

int16_t* MPU_6050_Get_Raw_XYZ_Gyroscope();

float* MPU_6050_Get_Adjusted_XYZ_Gyroscope();

#endif /* MPU_6050_H_ */
