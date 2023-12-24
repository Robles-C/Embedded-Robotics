/**
 * @file Final_Project_main.c
 *
 * @brief Main source code for the Final Project.
 *
 * @author Cristian Robles David Torres
 *
 */

#include <stdint.h>
#include <math.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/GPIO.h"
#include "../inc/EUSCI_A0_UART.h"
#include "../inc/Bumper_Sensors.h"
#include "../inc/Timer_A0_PWM.h"
#include "../inc/Timer_A3_Capture.h"
#include "../inc/Motor.h"
#include "../inc/Tachometer.h"


#define STDBOOL_H

#define bool _Bool
#define false 0
#define true 1

// Initialize length of the tachometer buffers
#define BUFFER_LENGTH                 10

// Global variable used to store the current state of the bumper sensors when an interrupt
// occurs (Bumper_Sensors_Handler). It will get updated on each interrupt event.
uint8_t bumper_sensor_value         = 0;

// Global variable that gets set in Bumper_Sensors_Handler.
// This is used to detect if any collisions occurred
bool collision_detected          = 0;

// Number of left wheel steps measured by the tachometer
int32_t Left_Steps                  = 0;

// Number of right wheel steps measured by the tachometer
int32_t Right_Steps                 = 0;

uint8_t rState = 100;

// Number of left wheel inches measured by the tachometer
float Left_Inches                  = 0;

// Number of right wheel inches measured by the tachometer
float Right_Inches                 = 0;

//PID Error
float error = 0;
//constants
float pC[3] = {7, .1, 2};

uint8_t current_bumper_sensor_state = 0;

// Store tachometer period of the left wheel in a buffer
// Number of 83.3 ns clock cycles to rotate 1/360 of a wheel rotation
uint16_t Tachometer_Buffer_Left[BUFFER_LENGTH];

// Store tachometer period of the right wheel in a buffer
// Number of 83.3 ns clock cycles to rotate 1/360 of a wheel rotation
uint16_t Tachometer_Buffer_Right[BUFFER_LENGTH];

// Direction of the left wheel's rotation
enum Tachometer_Direction Left_Direction;

// Direction of the right wheel's rotation
enum Tachometer_Direction Right_Direction;

// PID Control struct
typedef struct {
    float Kp; // Proportional Gain
    float Ki; // Integral Gain
    float Kd; // Derivative Gain
    float integral; // Integral sum
    float prev_error; // Previous error
} PID_Controller_t;

// Global PID controller instances
PID_Controller_t leftWheelPID, rightWheelPID;

/**
 * @brief Bumper sensor interrupt handler function.
 *
 * This is the interrupt handler for the bumper sensor interrupts. It is called when a falling edge event is detected on
 * any of the bumper sensor pins. The function checks if a collision has already been detected; if not, it prints a collision
 * detection message along with the bumper sensor state and sets a collision flag to prevent further detections.
 *
 * @param bumper_sensor_state An 8-bit unsigned integer representing the bump sensor states at the time of the interrupt.
 *
 * @return None
 */
void Bumper_Sensors_Handler(uint8_t bumper_sensor_state)
{
    if (collision_detected == 0)
    {
        current_bumper_sensor_state = bumper_sensor_state;

        //printf("New Command: %d %d\n", bumper_sensor_state, current_bumper_sensor_state);
        collision_detected = 1;
    }
}

/**
 * @brief Initialized PID controller
 *
 * @return None
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0;
    pid->prev_error = 0.0;
}

/**
 * @brief PID Control Math function.
 *
 * @return error * Kp + integral * Ki + derivative * Kd
 */
float PID_Controller(PID_Controller_t *pid, float setpoint, float actual_value) {
    float error = setpoint - actual_value;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    pid->prev_error = error;

    return (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
}

/**
 * @brief inputs parameters into the motor_forward function
 *
 * @return None
 */
void moveDrive(float controlSignalLeft, float controlSignalRight){
    if(controlSignalLeft > 0 && controlSignalRight > 0){//both need to move forward
        Motor_Forward(controlSignalLeft, controlSignalRight*.99);
    }else if(controlSignalLeft < 0 && controlSignalRight < 0){//both need to move backward
        Motor_Backward(-controlSignalLeft, -controlSignalRight*.99);
    }else if(controlSignalLeft < -10 && controlSignalRight > 10){// turning right
        Motor_Left(controlSignalLeft,controlSignalRight);
    }else if(controlSignalLeft > 10 && controlSignalRight < -10){// turning left
        Motor_Right(-controlSignalLeft, controlSignalRight);
    }else if(controlSignalLeft == 0 && controlSignalRight == 0){// no move
        Motor_Forward(0, 0);
    }
}

/**
 * @brief updates current position of robot using the tachometer values. 1 rotation of wheel is 360
 * tachometer ticks. The diameter of the wheel is about 2.73 inches, radius = half that. using the
 * circumference formula 2*pi*r we can find that the circumference is 8.6 (1.37*2*3.14). this value
 * this value is multiplied by left steps then divided by 360 ticks to determine how many inches the
 * robot has driven.
 *
 * @return None
 */
void updateCurrPos(){
    int i = 0;
    Tachometer_Get(&Tachometer_Buffer_Left[i], &Left_Direction, &Left_Steps, &Tachometer_Buffer_Right[i], &Right_Direction, &Right_Steps);

    Left_Inches = Left_Steps * 8.6 / 360;
    Right_Inches = Right_Steps * 8.6 / 360;

    //printf("%f, %f \n",  Left_Inches, Right_Inches);
}

/**
 * @brief Calculate the average of both wheel rpms
 *
 * Used to determine if the robot is still moving. If control signal is greater than a certain value then
 * robot is still moving.
 *
 * @param controlSignal right and left
 *
 * @return None
 */
bool isDriveMoving(float controlSignalRight, float controlSignalLeft){
    if(((abs(controlSignalRight + controlSignalLeft))/2) < 200){
        return 0;
    }else{
        return 1;
    }
}

/**
 * @brief command handler. Acts upon the value that is updated once the bumper switch has been pressed.
 * The updated value is rState. When rstate is 1 robot moves 10 inches forward with pid controller
 * when rstate is 2 robot moves 10 inches backward with pid controller
 *
 * @return None
 */
void Motor_Command_Handler() {

    updateCurrPos();

    float setpointLeft, setpointRight, controlSignalLeft, controlSignalRight;

    switch(rState) {
       case 1: // forward
           setpointLeft = 10;
           setpointRight = 10;

           controlSignalLeft = PID_Controller(&leftWheelPID, setpointLeft, Left_Inches);
           controlSignalRight = PID_Controller(&rightWheelPID, setpointRight, Right_Inches);
           //printf("Left_Error: %f | Right_Error: %f\n", controlSignalLeft, controlSignalRight);
           LED2_Output(RGB_LED_BLUE);
           moveDrive(controlSignalLeft, controlSignalRight);

           //printf("forward %f, %f \n", setpointLeft,  Left_Inches);
           if((fabs(Left_Inches - setpointLeft) < .25) && (fabs(Right_Inches - setpointRight) < .25) && !isDriveMoving(controlSignalLeft, controlSignalRight)){
               Tachometer_Reset();
               rState = 0;
           }
           break;
       case 2: // backward
           setpointLeft = -10;
           setpointRight = -10;

           controlSignalLeft = PID_Controller(&leftWheelPID, setpointLeft, Left_Inches);
           controlSignalRight = PID_Controller(&rightWheelPID, setpointRight, Right_Inches);
           //printf("Left_Error: %f | Right_Error: %f\n", controlSignalLeft, controlSignalRight);
           LED2_Output(RGB_LED_RED);
           moveDrive(controlSignalLeft, controlSignalRight);

           //printf("backward %f, %f \n", setpointLeft,  Left_Inches);
           if((fabs(Left_Inches - setpointLeft) < .25) && (fabs(Right_Inches - setpointRight) < .25) && !isDriveMoving(controlSignalLeft, controlSignalRight)){
               Tachometer_Reset();
               rState = 0;
           }
           break;
       case 3: // turn left
           setpointLeft = -4.31;
           setpointRight = 4.31;

           controlSignalLeft = PID_Controller(&leftWheelPID, setpointLeft, Left_Inches);
           controlSignalRight = PID_Controller(&rightWheelPID, setpointRight, Right_Inches);
           printf("Left_Error: %f | Right_Error: %f\n", controlSignalLeft, controlSignalRight);
           LED2_Output(RGB_LED_YELLOW);
           moveDrive(controlSignalLeft, controlSignalRight);

           //printf("turn left %f, %f \n", setpointLeft,  Left_Inches);
           if((fabs(Left_Inches - setpointLeft) < .5) && (fabs(Right_Inches - setpointRight) < .5) && !isDriveMoving(controlSignalLeft, controlSignalRight)){
               Tachometer_Reset();
               rState = 0;
           }
           break;
       case 4: // turn right
           setpointLeft = 4.31;
           setpointRight = -4.31;

           controlSignalLeft = PID_Controller(&leftWheelPID, setpointLeft, Left_Inches);
           controlSignalRight = PID_Controller(&rightWheelPID, setpointRight, Right_Inches);
           printf("Left_Error: %f | Right_Error: %f\n", controlSignalLeft, controlSignalRight);
           LED2_Output(RGB_LED_PINK);
           moveDrive(controlSignalLeft, controlSignalRight);

           //printf("%f, %f \n", Right_Inches,  Left_Inches);
           if((fabs(Left_Inches - setpointLeft) < .5) && (fabs(Right_Inches - setpointRight) < .5) && !isDriveMoving(controlSignalLeft, controlSignalRight)){
               Tachometer_Reset();
               rState = 0;
           }
           break;
       default :
           setpointLeft = 0;
           setpointRight = 0;
           LED2_Output(RGB_LED_GREEN);
           controlSignalLeft = PID_Controller(&leftWheelPID, setpointLeft, Left_Inches);
           controlSignalRight = PID_Controller(&rightWheelPID, setpointRight, Right_Inches);
           moveDrive(controlSignalLeft, controlSignalRight);
    }
}

/**
 * @brief once the handle collision has happened this function determines what button was pressed and
 * sends a command to the command_motor_handler function
 *
 * @return None
 */
void Handle_Collision()
{
    switch(current_bumper_sensor_state) {
        case 1://right most bumper sensor
            //right turn
            rState = 4;
            Clock_Delay1ms(500);
            current_bumper_sensor_state = 0;
            collision_detected = 0;
            break;
        case 2:
            //

            Clock_Delay1ms(500);
            current_bumper_sensor_state = 0;
            collision_detected = 0;
            break;
        case 4:
            //forward

            rState = 1;
            Clock_Delay1ms(500);
            current_bumper_sensor_state = 0;
            collision_detected = 0;
            break;
        case 8:
            //backward
            rState = 2;
            Clock_Delay1ms(500);
            current_bumper_sensor_state = 0;
            collision_detected = 0;
            break;
        case 16:
            //
            Clock_Delay1ms(500);
            current_bumper_sensor_state = 0;
            collision_detected = 0;
            break;
        case 32://left most bumper sensor
            //left turn
            rState = 3;
            Clock_Delay1ms(500);
            current_bumper_sensor_state = 0;
            collision_detected = 0;
            break;
        default:
            //printf("not a valid input\n");
            collision_detected = 0;
        }
}

int main(void)
{
    // Initialize collision_detected flag
    collision_detected = 0;

    // Ensure that interrupts are disabled during initialization
    DisableInterrupts();

    // Initialize the 48 MHz Clock
    Clock_Init48MHz();

    // Initialize the built-in red LED
    LED1_Init();
    LED2_Init();

    // Initialize the front and back LEDs
    P8_Init();

    // Initialize the buttons
    Buttons_Init();

    // Initialize EUSCI_A0_UART
    EUSCI_A0_UART_Init_Printf();

    // Initialize the bumper sensors which will be used to generate external I/O-triggered interrupts
    Bumper_Sensors_Init(&Bumper_Sensors_Handler);

    // Initialize the tachometers
    Tachometer_Init();

    // Initialize the motors
    Motor_Init();

    // Initialize PID Controllers
    PID_Init(&leftWheelPID, pC[0] /*Kp*/, pC[1]/*Ki*/, pC[2] /*Kd*/); // Tune these values
    PID_Init(&rightWheelPID, pC[0] /*Kp*/, pC[1] /*Ki*/, pC[2] /*Kd*/); // Tune these values

    // Enable the interrupts used by the modules
    EnableInterrupts();

    while(1)
    {
        while (collision_detected == 0){
            //prints for testing
            //printf("Desired_RPM_Left: %d | Desired_RPM_Right: %d\n", Desired_RPM_Left, Desired_RPM_Right);
            //printf("Actual_RPM_Left: %d | Actual_RPM_Right: %d\n", Actual_RPM_Left, Actual_RPM_Right);
            //printf("Left_Inches: %f | Right Inches: %f\n\n", Left_Inches, Right_Inches);
            printf("%i\n", rState);
            Motor_Command_Handler();
            Clock_Delay1ms(50);
        }

        while (collision_detected == 1){
            Handle_Collision();
        }

    }
}
