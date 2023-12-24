/**
 * @file MPU_6050.c
 * @brief Source code for the MPU_6050 driver.
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

#include "../inc/MPU_6050.h"

void MPU_6050_Write_Register(uint8_t register_address, uint8_t register_data)
{
    uint8_t buffer[] =
    {
         register_address,
         register_data & 0xFF
    };

    EUSCI_B1_I2C_Send_Multiple_Bytes(MPU_6050_ADDRESS, buffer, sizeof(buffer));
}

uint8_t MPU_6050_Read_Register(uint8_t register_address)
{
    EUSCI_B1_I2C_Send_A_Byte(MPU_6050_ADDRESS, register_address);
    uint8_t received_data = EUSCI_B1_I2C_Receive_A_Byte(MPU_6050_ADDRESS);
    return received_data;
}

void MPU_6050_Init()
{
    // Initialize I2C using EUSCI_B1 module
    EUSCI_B1_I2C_Init();

    // Reset the MPU-6050 Accelerometer and Gyroscope sensor
    MPU_6050_Reset();

    // Set CLKSEL config to 0x01: PLL with X axis gyroscope reference
    MPU_6050_Set_Clock_Source(CLKSEL_PLL_X_AXIS_GYRO_REFERENCE);

    // Set sample rate value to default
    MPU_6050_Set_Sample_Rate_Divider(0x00);

    // Set Digital Low-Pass Filter Bandwidth to 21 Hz
    MPU_6050_Set_DLPF_Bandwidth(MPU_6050_DLPF_CFG_BANDWIDTH_21_HZ);

    // Set accelerometer range to +/- 8g
    MPU_6050_Set_Accelerometer_Range(MPU_6050_ACCEL_RANGE_8_G);

    // Set gyroscope range to +/- 500 degrees/second
    MPU_6050_Set_Gyroscope_Range(MPU_6050_GYRO_RANGE_500_DEG);
}

void MPU_6050_Reset()
{
    MPU_6050_Write_Register(MPU_6050_PWR_MGMT_1, MPU_REG_SET_BIT_7);
    Clock_Delay1ms(100);

    MPU_6050_Write_Register(MPU_6050_SIGNAL_PATH_RESET, MPU_REG_SET_BIT_2 | MPU_REG_SET_BIT_1 | MPU_REG_SET_BIT_0);
    Clock_Delay1ms(100);
}

uint8_t MPU_6050_Get_Device_ID()
{
    uint8_t MPU_6050_Device_ID = MPU_6050_Read_Register(MPU_6050_WHO_AM_I);
    return MPU_6050_Device_ID;
}

void MPU_6050_Set_Clock_Source(uint8_t clock_source_select)
{
    MPU_6050_Write_Register(MPU_6050_PWR_MGMT_1, clock_source_select);
    Clock_Delay1ms(100);
}

uint8_t MPU_6050_Get_Clock_Source()
{
    uint8_t MPU_6050_Clock_Source = MPU_6050_Read_Register(MPU_6050_PWR_MGMT_1);

    // Bits 2-0 f the PWR_MGMT_1 register represent CLKSEL[2:0]
    MPU_6050_Clock_Source = (MPU_6050_Clock_Source & (MPU_REG_SET_BIT_2 | MPU_REG_SET_BIT_1 | MPU_REG_SET_BIT_0));

    return MPU_6050_Clock_Source;
}

void MPU_6050_Set_Sample_Rate_Divider(uint8_t sample_rate_divider)
{
    MPU_6050_Write_Register(MPU_6050_SMPRT_DIV, sample_rate_divider);
}

uint8_t MPU_6050_Get_Sample_Rate_Divider()
{
    uint8_t MPU_6050_Sample_Rate_Divider = MPU_6050_Read_Register(MPU_6050_SMPRT_DIV);

    return MPU_6050_Sample_Rate_Divider;
}

void MPU_6050_Set_DLPF_Bandwidth(uint8_t dlpf_cfg)
{
    MPU_6050_Write_Register(MPU_6050_CONFIG, dlpf_cfg);
}

uint8_t MPU_6050_Get_DLPF_Bandwidth()
{
    uint8_t MPU_6050_DLPF_CFG = MPU_6050_Read_Register(MPU_6050_CONFIG);

    // Bits 2-0 of the ACONFIG register represent DLPF_CFG[2:0]
    MPU_6050_DLPF_CFG = (MPU_6050_DLPF_CFG & (MPU_REG_SET_BIT_2 | MPU_REG_SET_BIT_1 | MPU_REG_SET_BIT_0));

    return MPU_6050_DLPF_CFG;
}

void MPU_6050_Set_Accelerometer_Range(uint8_t accelerometer_range)
{
    MPU_6050_Write_Register(MPU_6050_ACCEL_CONFIG, accelerometer_range);
}

uint8_t MPU_6050_Get_Accelerometer_Range()
{
    uint8_t MPU_6050_Accelerometer_Range = MPU_6050_Read_Register(MPU_6050_ACCEL_CONFIG);

    // Bit 4 and Bit 3 of the ACCEL_CONFIG register represents AFS_SEL[1:0]
    MPU_6050_Accelerometer_Range = (MPU_6050_Accelerometer_Range & (MPU_REG_SET_BIT_4 | MPU_REG_SET_BIT_3));

    return MPU_6050_Accelerometer_Range;
}

int MPU_6050_Get_Accelerometer_Scale(uint8_t accelerometer_range)
{
    int accelerometer_scale;

    switch(accelerometer_range)
    {
        case MPU_6050_ACCEL_RANGE_2_G:
        {
            accelerometer_scale = 16384;
            break;
        }

        case MPU_6050_ACCEL_RANGE_4_G:
        {
            accelerometer_scale = 8192;
            break;
        }

        case MPU_6050_ACCEL_RANGE_8_G:
        {
            accelerometer_scale = 4096;
            break;
        }

        case MPU_6050_ACCEL_RANGE_16_G:
        {
            accelerometer_scale = 2048;
            break;
        }
    }
    return accelerometer_scale;
}

uint8_t MPU_6050_Get_Accel_X_High_Byte()
{
    uint8_t Accel_X_High_Byte = MPU_6050_Read_Register(MPU_6050_ACCEL_XOUT_H);
    return Accel_X_High_Byte;
}

uint8_t MPU_6050_Get_Accel_X_Low_Byte()
{
    uint8_t Accel_X_Low_Byte = MPU_6050_Read_Register(MPU_6050_ACCEL_XOUT_L);
    return Accel_X_Low_Byte;
}

uint8_t MPU_6050_Get_Accel_Y_High_Byte()
{
    uint8_t Accel_Y_High_Byte = MPU_6050_Read_Register(MPU_6050_ACCEL_YOUT_H);
    return Accel_Y_High_Byte;
}

uint8_t MPU_6050_Get_Accel_Y_Low_Byte()
{
    uint8_t Accel_Y_Low_Byte = MPU_6050_Read_Register(MPU_6050_ACCEL_YOUT_L);
    return Accel_Y_Low_Byte;
}

uint8_t MPU_6050_Get_Accel_Z_High_Byte()
{
    uint8_t Accel_Z_High_Byte = MPU_6050_Read_Register(MPU_6050_ACCEL_ZOUT_H);
    return Accel_Z_High_Byte;
}

uint8_t MPU_6050_Get_Accel_Z_Low_Byte()
{
    uint8_t Accel_Z_Low_Byte = MPU_6050_Read_Register(MPU_6050_ACCEL_ZOUT_L);
    return Accel_Z_Low_Byte;
}

int16_t* MPU_6050_Get_Raw_XYZ_Acceleration()
{
    static int16_t raw_acceleration_buffer[3];

    uint8_t Raw_Acceleration_X_High_Byte = MPU_6050_Get_Accel_X_High_Byte();
    uint8_t Raw_Acceleration_X_Low_Byte = MPU_6050_Get_Accel_X_Low_Byte();
    int16_t Raw_Acceleration_X = (Raw_Acceleration_X_High_Byte << 8) | Raw_Acceleration_X_Low_Byte;

    uint8_t Raw_Acceleration_Y_High_Byte = MPU_6050_Get_Accel_Y_High_Byte();
    uint8_t Raw_Acceleration_Y_Low_Byte = MPU_6050_Get_Accel_Y_Low_Byte();
    int16_t Raw_Acceleration_Y = (Raw_Acceleration_Y_High_Byte << 8) | Raw_Acceleration_Y_Low_Byte;

    uint8_t Raw_Acceleration_Z_High_Byte = MPU_6050_Get_Accel_Z_High_Byte();
    uint8_t Raw_Acceleration_Z_Low_Byte = MPU_6050_Get_Accel_Z_Low_Byte();
    int16_t Raw_Acceleration_Z = (Raw_Acceleration_Z_High_Byte << 8) | Raw_Acceleration_Z_Low_Byte;

    raw_acceleration_buffer[0] = Raw_Acceleration_X;
    raw_acceleration_buffer[1] = Raw_Acceleration_Y;
    raw_acceleration_buffer[2] = Raw_Acceleration_Z;

    return raw_acceleration_buffer;
}

float* MPU_6050_Get_Adjusted_XYZ_Acceleration()
{
    int16_t* raw_acceleration_buffer;
    static float acceleration_buffer[3];

    uint8_t accelerometer_range = MPU_6050_Get_Accelerometer_Range();
    int accelerometer_scale = MPU_6050_Get_Accelerometer_Scale(accelerometer_range);

    raw_acceleration_buffer = MPU_6050_Get_Raw_XYZ_Acceleration();

    float Acceleration_X = ((float)(raw_acceleration_buffer[0])) / accelerometer_scale;
    float Acceleration_Y = ((float)(raw_acceleration_buffer[1])) / accelerometer_scale;
    float Acceleration_Z = ((float)(raw_acceleration_buffer[2])) / accelerometer_scale;

    acceleration_buffer[0] = Acceleration_X;
    acceleration_buffer[1] = Acceleration_Y;
    acceleration_buffer[2] = Acceleration_Z;

    return acceleration_buffer;
}

void MPU_6050_Set_Gyroscope_Range(uint8_t gyroscope_range)
{
    MPU_6050_Write_Register(MPU_6050_GYRO_CONFIG, gyroscope_range);
}

uint8_t MPU_6050_Get_Gyroscope_Range()
{
    uint8_t MPU_6050_Gyroscope_Range = MPU_6050_Read_Register(MPU_6050_GYRO_CONFIG);

    // Bit 4 and Bit 3 of the GYRO_CONFIG register represents FS_SEL[1:0]
    MPU_6050_Gyroscope_Range = (MPU_6050_Gyroscope_Range & (MPU_REG_SET_BIT_4 | MPU_REG_SET_BIT_3));

    return MPU_6050_Gyroscope_Range;
}

float MPU_6050_Get_Gyroscope_Scale(uint8_t gyroscope_range)
{
    float gyroscope_scale;

    switch(gyroscope_range)
    {
        case MPU_6050_GYRO_RANGE_250_DEG:
        {
            gyroscope_scale = 131.0;
            break;
        }

        case MPU_6050_GYRO_RANGE_500_DEG:
        {
            gyroscope_scale = 65.5;
            break;
        }

        case MPU_6050_GYRO_RANGE_1000_DEG:
        {
            gyroscope_scale = 32.8;
            break;
        }

        case MPU_6050_GYRO_RANGE_2000_DEG:
        {
            gyroscope_scale = 16.4;
            break;
        }
    }
    return gyroscope_scale;
}

uint8_t MPU_6050_Get_Gyro_X_High_Byte()
{
    uint8_t Gyro_X_High_Byte = MPU_6050_Read_Register(MPU_6050_GYRO_XOUT_H);
    return Gyro_X_High_Byte;
}

uint8_t MPU_6050_Get_Gyro_X_Low_Byte()
{
    uint8_t Gyro_X_Low_Byte = MPU_6050_Read_Register(MPU_6050_GYRO_XOUT_L);
    return Gyro_X_Low_Byte;
}

uint8_t MPU_6050_Get_Gyro_Y_High_Byte()
{
    uint8_t Gyro_Y_High_Byte = MPU_6050_Read_Register(MPU_6050_GYRO_YOUT_H);
    return Gyro_Y_High_Byte;
}

uint8_t MPU_6050_Get_Gyro_Y_Low_Byte()
{
    uint8_t Gyro_Y_Low_Byte = MPU_6050_Read_Register(MPU_6050_GYRO_YOUT_L);
    return Gyro_Y_Low_Byte;
}

uint8_t MPU_6050_Get_Gyro_Z_High_Byte()
{
    uint8_t Gyro_Z_High_Byte = MPU_6050_Read_Register(MPU_6050_GYRO_ZOUT_H);
    return Gyro_Z_High_Byte;
}

uint8_t MPU_6050_Get_Gyro_Z_Low_Byte()
{
    uint8_t Gyro_Z_Low_Byte = MPU_6050_Read_Register(MPU_6050_GYRO_ZOUT_L);
    return Gyro_Z_Low_Byte;
}

int16_t* MPU_6050_Get_Raw_XYZ_Gyroscope()
{
    static int16_t raw_gyroscope_buffer[3];

    uint8_t Raw_Gyroscope_X_High_Byte = MPU_6050_Get_Gyro_X_High_Byte();
    uint8_t Raw_Gyroscope_X_Low_Byte = MPU_6050_Get_Gyro_X_Low_Byte();
    int16_t Raw_Gyroscope_X = (Raw_Gyroscope_X_High_Byte << 8) | Raw_Gyroscope_X_Low_Byte;

    uint8_t Raw_Gyroscope_Y_High_Byte = MPU_6050_Get_Gyro_Y_High_Byte();
    uint8_t Raw_Gyroscope_Y_Low_Byte = MPU_6050_Get_Gyro_Y_Low_Byte();
    int16_t Raw_Gyroscope_Y = (Raw_Gyroscope_Y_High_Byte << 8) | Raw_Gyroscope_Y_Low_Byte;

    uint8_t Raw_Gyroscope_Z_High_Byte = MPU_6050_Get_Gyro_Z_High_Byte();
    uint8_t Raw_Gyroscope_Z_Low_Byte = MPU_6050_Get_Gyro_Z_Low_Byte();
    int16_t Raw_Gyroscope_Z = (Raw_Gyroscope_Z_High_Byte << 8) | Raw_Gyroscope_Z_Low_Byte;

    raw_gyroscope_buffer[0] = Raw_Gyroscope_X;
    raw_gyroscope_buffer[1] = Raw_Gyroscope_Y;
    raw_gyroscope_buffer[2] = Raw_Gyroscope_Z;

    return raw_gyroscope_buffer;
}

float* MPU_6050_Get_Adjusted_XYZ_Gyroscope()
{
    int16_t* raw_gyroscope_buffer;
    static float gyroscope_buffer[3];

    uint8_t gyroscope_range = MPU_6050_Get_Gyroscope_Range();
    int gyroscope_scale = MPU_6050_Get_Gyroscope_Scale(gyroscope_range);

    raw_gyroscope_buffer = MPU_6050_Get_Raw_XYZ_Gyroscope();

    float Gyroscope_X = ((float)(raw_gyroscope_buffer[0])) / gyroscope_scale;
    float Gyroscope_Y = ((float)(raw_gyroscope_buffer[1])) / gyroscope_scale;
    float Gyroscope_Z = ((float)(raw_gyroscope_buffer[2])) / gyroscope_scale;

    gyroscope_buffer[0] = Gyroscope_X;
    gyroscope_buffer[1] = Gyroscope_Y;
    gyroscope_buffer[2] = Gyroscope_Z;

    return gyroscope_buffer;
}
