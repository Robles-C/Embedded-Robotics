/**
 * @file PMOD_ALS.c
 * @brief Source code for the PMOD_ALS driver.
 *
 * This file contains the function definitions for the PMOD_ALS driver.
 * It interfaces with the PMOD ALS module, which uses the SPI communication protocol.
 *  - Product Link: https://digilent.com/shop/pmod-als-ambient-light-sensor/
 *  - Reference Manual: https://digilent.com/reference/pmod/pmodals/reference-manual
 *
 * The PMOD ALS module uses the following SPI configuration:
 *  - SPI Mode 3
 *  - Chip Select Active Low
 *  - SCLK Frequency: 1 MHz
 *  - MSB First
 *
 * The following connections must be made:
 *  - PMOD ALS CS       (Pin 1)     <-->  MSP432 LaunchPad Pin P9.4 (CS)
 *  - PMOD ALS NC       (Pin 2)     <-->  Not Connected
 *  - PMOD ALS MISO     (Pin 3)     <-->  MSP432 LaunchPad Pin P9.6 (MISO)
 *  - PMOD ALS SCLK     (Pin 4)     <-->  MSP432 LaunchPad Pin P9.5 (SCLK)
 *  - PMOD ALS GND      (Pin 5)     <-->  MSP432 LaunchPad GND
 *  - PMOD ALS VCC      (Pin 6)     <-->  MSP432 LaunchPad VCC (3.3V)
 *
 * A value of 0 indicates a low light level and a value of 255 indicates a high light level.
 *
 * For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/PMOD_ALS.h"

// Initialize global buffer to store received data from the PMOD JSTK2 module
uint8_t light_intensity_buffer[8] = {0};

void PMOD_ALS_Init()
{
    // Hold the EUSCI_A3 module in reset mode
    EUSCI_A3->CTLW0 |= 0x01;

//     CTWL0 Register Configuration
//
//      Bit(s)      Field       Value       Description
//      -----       -----       -----       -----------
//       15         UCCKPH       0x0        Data is shifted out on the falling edge of SCLK and sampled on the rising edge of SCLK
//       14         UCCKPL       0x1        Clock is logic high when inactive
//       13         UCMSB        0x1        MSB first
//       12         UC7BIT       0x0        8-bit data
//       11         UCMST        0x1        Master mode is selected
//       10-9       UCMODEx      0x2        4-pin SPI with active low UCSTE
//       8          UCSYNC       0x1        Synchronous mode
//       7-6        UCSSELx      0x2        eUSCI clock source is SMCLK
//       5-2        Reserved     0x0        Reserved
//       1          UCSTEM       0x1        UCSTE pin is used to generate signal for 4-wire slave
//       0          UCSWRST      0x1        eUSCI logic held in reset state
    EUSCI_A3->CTLW0 |= 0x6D83;

    // Set the baud rate. The clock frequency used is 1 MHz.
    // N = (Clock Frequency) / (Baud Rate) = (12,000,000 / 1,000,000)
    // N = 12
    EUSCI_A3->BRW |= 12;

    // Configure P9.5, P9.6, and P9.7 pins as primary module function
    P9->SEL0 |= 0xE0;
    P9->SEL1 &= ~0xE0;

    // Configure P9.4 as output GPIO (Chip Select)
    P9->SEL0 &= ~0x10;
    P9->SEL1 &= ~0x10;
    P9->DIR |= 0x10;

    // Drive P9.4 high (Chip Select is active low)
    P9->OUT |= 0x10;

    // Clear the software reset bit to enable the EUSCI_A3 module
    EUSCI_A3->CTLW0 &= ~0x01;

    // Ensure that the following interrupts are disabled:
    // - Receive Interrupt
    // - Transmit Interrupt
    EUSCI_A3->IE &= ~0x03;
}

void PMOD_ALS_Chip_Select(uint8_t chip_select_enable)
{
    // Clear P9.4 to 0 when chip select is enabled
    if (chip_select_enable == 0x00)
    {
        P9->OUT &= ~0x10;
    }

    // Set P9.4 to 1 when chip select is disabled
    else
    {
        P9->OUT |= 0x10;
    }
}

void PMOD_ALS_Write_SPI_Data(uint8_t data)
{
    // Wait until UCA3TXBUF is empty
    while((EUSCI_A3->IFG & 0x0002) == 0x0000);

    // Write the data byte to the transmit buffer
    EUSCI_A3->TXBUF = data;
}

uint8_t PMOD_ALS_Read_SPI_Data()
{
    // Wait until UCA3RXBUF is empty
    while((EUSCI_A3->IFG & 0x0001) == 0x0000);

    // Retrieve received data from the RX buffer and write to a local variable
    uint8_t data = EUSCI_A3->RXBUF;

    // Return received data
    return data;
}

uint8_t PMOD_ALS_Read_Light_Intensity()
{
    // Enable the chip select line
    PMOD_ALS_Chip_Select(0x00);

    for (int i = 0; i < 2; i++)
    {
        // Write dummy bytes (0x00) to the TX buffer and transmit it to generate SCLK pulses
        PMOD_ALS_Write_SPI_Data(0x00);

        // Receive data from the PMOD ALS module and write it to the RX buffer
        light_intensity_buffer[i] = PMOD_ALS_Read_SPI_Data();
    }

    // Disable the chip select line
    PMOD_ALS_Chip_Select(0x01);

    // Extract the 8-bit data from the 16-bit data read from the PMOD ALS
    uint8_t light_intensity = (light_intensity_buffer[0] << 3) | (light_intensity_buffer[1] >> 5);

    // Return the measured light intensity
    return light_intensity;
}
