/**
 * @file PMOD_JSTK2.h
 * @brief Header file for the PMOD_JSTK2 driver.
 *
 * This file contains the function definitions for the PMOD_JSTK2 driver.
 * It interfaces with the PMOD JSTK2 module, which uses the SPI communication protocol.
 *  - Product Link: https://digilent.com/shop/pmod-jstk2-two-axis-joystick/
 *  - Reference Manual: https://digilent.com/reference/pmod/pmodjstk2/reference-manual
 *
 * The PMOD JSTK2 module uses the following SPI configuration:
 *  - SPI Mode 0
 *  - Chip Select Active Low
 *  - SCLK Frequency: 1 MHz
 *  - MSB First
 *
 * The following connections must be made:
 *  - PMOD JSTK2 CS     (Pin 1)     <-->  MSP432 LaunchPad Pin P9.4 (CS)
 *  - PMOD JSTK2 MOSI   (Pin 2)     <-->  MSP432 LaunchPad Pin P9.7 (MOSI)
 *  - PMOD JSTK2 MISO   (Pin 3)     <-->  MSP432 LaunchPad Pin P9.6 (MISO)
 *  - PMOD JSTK2 SCLK   (Pin 4)     <-->  MSP432 LaunchPad Pin P9.5 (SCLK)
 *  - PMOD JSTK2 GND    (Pin 5)     <-->  MSP432 LaunchPad GND
 *  - PMOD JSTK2 VCC    (Pin 6)     <-->  MSP432 LaunchPad VCC (3.3V)
 *
 * For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#ifndef PMOD_JSTK2_H_
#define PMOD_JSTK2_H_

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"

// Define dummy byte to send to the PMOD JSTK2 module
#define JSTK2_DUMMY_BYTE                0x00

// Define commands used to send to the PMOD JSTK2 module
#define JSTK2_CMD_GET_RAW               0x00
#define JSTK2_CMD_SET_LED               0x80
#define JSTK2_CMD_SET_RGB_LED           0x84
#define JSTK2_CMD_GET_POSITION          0xC0

// Declare global buffer to store received data from the PMOD JSTK2 module
extern uint8_t rx_buffer[10];

// Declare global variables to store the fsButtons status:
//  - Joystick Center Button (Bit 0)
//  - Trigger Button (Bit 1)
//
//  1 = Pressed
//  0 = Not pressed
extern uint8_t Center_Button_Status;
extern uint8_t Trigger_Button_Status;

// Declare global variables to store 10-bit X and Y position values
extern int X_Position_10_Bit;
extern int Y_Position_10_Bit;

// Declare global variables to store 8-bit X and Y position values
extern uint8_t X_Position_8_Bit;
extern uint8_t Y_Position_8_Bit;

/**
 * @brief Initializes the SPI module EUSCI_A3 for the PMOD JSTK2 module.
 *
 * This function configures the EUSCI_A3 module to enable SPI communication
 * for the PMOD JSTK2 module with the following configuration:
 *
 * - CTLW0 Register Configuration:
 *
 *  Bit(s)      Field       Value       Description
 *  -----       -----       -----       -----------
 *   15         UCCKPH       0x1        Data is captured on the first edge and changed on the following edge
 *   14         UCCKPL       0x0        Clock is low when inactive
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
void PMOD_JSTK2_Init();


/**
 * @brief This function controls the Chip Select (CS) line for the PMOD JSTK2 module.
 *
 * This function controls the Chip Select (CS) line for the PMOD JSTK2 module.
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
void PMOD_JSTK2_Chip_Select(uint8_t chip_select_enable);

/**
 * @brief Transmit a single byte of data over SPI for the PMOD JSTK2 module.
 *
 * This function transmits a single byte of data over the Serial Peripheral Interface (SPI)
 * for the PMOD JSTK2 module. It writes the data to the transmit buffer, initiating
 * the SPI communication.
 *
 * @param data - The byte of data to be transmitted over SPI.
 *
 * @return None
 */
void PMOD_JSTK2_Write_SPI_Data(uint8_t data);

/**
 * @brief Read a byte of data received over SPI for the PMOD JSTK2 module.
 *
 * This function waits until the receive buffer (UCA3RXBUF) is not empty,
 * and then it retrieves the received data and returns it.
 *
 * @return uint8_t - The byte of data received over SPI.
 */
uint8_t PMOD_JSTK2_Read_SPI_Data();

/**
 * @brief Transmit a packet of data over SPI to the PMOD JSTK2 module and simultaneously receive data from it.
 *
 * This function enables the chip select line, includes delays for proper SPI communication,
 * iterates over the cmd_buffer, transmits data to the PMOD JSTK2 module,
 * and receives data from the PMOD JSTK2 module and writes it into rx_buffer.
 * Finally, it disables the chip select line and adds a delay for proper SPI communication termination.
 *
 * @param cmd_buffer      - Pointer to the command buffer to be transmitted.
 * @param buffer_length   - Length of the command buffer.
 *
 * @return None
 */
void PMOD_JSTK2_Transfer_Packet(uint8_t* cmd_buffer, uint8_t buffer_length);

/**
 * @brief Get the standard packet of data from the PMOD JSTK2 module.
 *
 * This function initializes a command buffer with zeros, writes the command to get standard data,
 * and then calls PMOD_JSTK2_Transfer_and_Receive_Packet to retrieve the 5-byte standard packet.
 *
 * @return None
 */
void PMOD_JSTK2_Get_Basic_Packet();

/**
 * @brief Set the RGB LED color for the PMOD JSTK2 module.
 *
 * This function initializes a command buffer, writes the command to set the RGB LED color,
 * and transmits the RGB color components. It uses PMOD_JSTK2_Transfer_and_Receive_Packet
 * to communicate with the module.
 *
 * @param red   - 8-bit value representing the red component of the RGB color.
 * @param green - 8-bit value representing the green component of the RGB color.
 * @param blue  - 8-bit value representing the blue component of the RGB color.
 *
 * @return None
 */
void PMOD_JSTK2_Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Get the X position of the joystick on the PMOD JSTK2 module.
 *
 * This function initializes a command buffer, writes the command to get the XY position,
 * and then calls PMOD_JSTK2_Transfer_and_Receive_Packet to retrieve the XY position data.
 * It returns the X position byte from the received data.
 *
 * @return uint8_t - The 8-bit X position of the joystick.
 */
uint8_t PMOD_JSTK2_Get_X_Position();

/**
 * @brief Get the Y position of the joystick on the PMOD JSTK2 module.
 *
 * This function initializes a command buffer, writes the command to get the XY position,
 * and then calls PMOD_JSTK2_Transfer_and_Receive_Packet to retrieve the XY position data.
 * It returns the Y position byte from the received data.
 *
 * @return uint8_t - The 8-bit Y position of the joystick.
 */
uint8_t PMOD_JSTK2_Get_Y_Position();

/**
 * @brief Get both X and Y positions of the joystick on the PMOD JSTK2 module.
 *
 * This function initializes a command buffer, writes the command to get the XY position,
 * and then calls PMOD_JSTK2_Transfer_and_Receive_Packet to retrieve the XY position data.
 * It assigns the 8-bit X and Y positions to the global variables, X_Position_8_Bit and Y_Position_8_Bit.
 *
 * @return None
 */
void PMOD_JSTK2_Get_XY_Position();

#endif /* PMOD_JSTK2_H_ */
