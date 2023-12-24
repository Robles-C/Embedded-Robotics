/**
 * @file PMOD_ALS.h
 * @brief Header file for the PMOD_ALS driver.
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

#ifndef PMOD_ALS_H_
#define PMOD_ALS_H_

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"

// Declare global buffer to store received data from the PMOD ALS module
extern uint8_t light_intensity_buffer[8];

/**
 * @brief Initializes the SPI module EUSCI_A3 for the PMOD ALS module.
 *
 * This function configures the EUSCI_A3 module to enable SPI communication
 * for the PMOD ALS module with the following configuration:
 *
 * - CTLW0 Register Configuration:
 *
 *  Bit(s)      Field       Value       Description
 *  -----       -----       -----       -----------
 *   15         UCCKPH       0x0        Data is shifted out on the falling edge of SCLK and sampled on the rising edge of SCLK
 *   14         UCCKPL       0x1        Clock is logic high when inactive
 *   13         UCMSB        0x1        MSB first
 *   12         UC7BIT       0x0        8-bit data
 *   11         UCMST        0x1        Master mode is selected
 *   10-9       UCMODEx      0x2        4-pin SPI with active low UCSTE
 *   8          UCSYNC       0x1        Synchronous mode
 *   7-6        UCSSELx      0x2        eUSCI clock source is SMCLK
 *   5-2        Reserved     0x0        Reserved
 *   1          UCSTEM       0x1        UCSTE pin is used to generate signal for 4-wire slave
 *   0          UCSWRST      0x1        eUSCI logic held in reset state
 *
 * - Clock Frequency: 1 MHz
 * - Interrupts disabled
 *
 * For more information regarding the registers used, refer to the eUSCI_A SPI Registers section (25.4)
 * of the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @return None
 */
void PMOD_ALS_Init();


/**
 * @brief This function controls the Chip Select (CS) line for the PMOD ALS module.
 *
 * This function controls the Chip Select (CS) line for the PMOD ALS module.
 * Depending on the provided chip_select_enable parameter, the function either enables or disables
 * the Chip Select line by manipulating the corresponding GPIO pin (P9.4) as follows:
 *
 * - When chip_select_enable is set to 0x00:
 *   - The function clears the corresponding GPIO pin (P9.4) to 0, enabling the Chip Select line.
 *
 * - When chip_select_enable is set to a value other than 0x00:
 *   - The function sets the corresponding GPIO pin (P9.4) to 1, disabling the Chip Select line.
 *
 * @param chip_select_enable    - Control parameter for Chip Select (CS) signal:
 *                              - 0x00: Enable Chip Select (CS) signal.
 *                              - Any other value: Disable Chip Select (CS) signal.
 *
 * @return None
 */
void PMOD_ALS_Chip_Select(uint8_t chip_select_enable);

/**
 * @brief Transmit a single byte of data over SPI for the PMOD ALS module.
 *
 * This function transmits a single byte of data over the Serial Peripheral Interface (SPI)
 * for the PMOD ALS module. It writes the data to the transmit buffer, initiating
 * the SPI communication.
 *
 * @param data - The byte of data to be transmitted over SPI.
 *
 * @return None
 */
void PMOD_ALS_Write_SPI_Data(uint8_t data);

/**
 * @brief Read a byte of data received over SPI for the PMOD ALS module.
 *
 * This function waits until the receive buffer (UCA3RXBUF) is not empty,
 * and then it retrieves the received data and returns it.
 *
 * @return uint8_t - The byte of data received over SPI.
 */
uint8_t PMOD_ALS_Read_SPI_Data();

/**
 * @brief Transmit two dummy bytes over SPI to the PMOD ALS module and simultaneously receive data from it.
 *
 * This function enables the chip select line, transmits dummy bytes to the PMOD ALS,
 * receives data from the PMOD ALS, and writes it into rx_buffer.
 * Finally, it disables the chip select line.
 *
 * @param cmd_buffer      - Pointer to the command buffer to be transmitted.
 * @param buffer_length   - Length of the command buffer.
 *
 * @return None
 */
uint8_t PMOD_ALS_Read_Light_Intensity();

#endif /* PMOD_ALS_H_ */
