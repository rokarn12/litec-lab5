// Lab 5
// Rojan Karn
// Proportional-Integral Control

///////////////////////////////////////////////////////////////////
// THIS LAB DEMONSTRATES A PROPORTIONAL-INTEGRAL CONTROL ROUTINE //
// - The desired "NORTH" direction is changed every 10 seconds,  //
//   and this PI control scheme allows for the car to quickly    //
//   adjust to this change                                       //
///////////////////////////////////////////////////////////////////

// Uses the compass to find the "NORTH" direction and turns the car towards that
// direction in full-speed forward. If the desired heading is more than 135 degrees
// away, the car will perform a k-turn

// No I/O for this lab

#include "C8051_SIM.h" // simulator header file
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// Inputs: 3 slideswitches
#define SS0 P3_5
#define SS1 P3_6
#define SS2 P3_7

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------
void Port_Init(void);
void PCA_Init (void);
void Interrupt_Init(void);
void XBR0_Init();
void SMB_Init();
void ReadCompass();
void ReadRanger();
void Set_Motor_Pulsewidth(void);
void Set_Servo_Pulsewidth(void);
void goNorth(void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
// Global variables for the important motor pulse widths
unsigned int MOTOR_CENTER = 2765; // 1.5 ms
unsigned int MOTOR_MIN = 2028; // 1.1 ms
unsigned int MOTOR_MAX = 3502; // 1.9 ms
unsigned int MOTOR_PW = 0;

// Do the same thing for servo pulse widths
unsigned int SERVO_CENTER = 2765; // 1.5 ms
unsigned int SERVO_MIN = 1659; // 0.9 ms
unsigned int SERVO_MAX = 3871; // 2.1 ms
unsigned int SERVO_PW = 0;

// use for 1 second delay in the beginning
unsigned char PCA_counts;

unsigned char Data[2] = {0, 0, 0};        // Data array for use with the SMB/I2C
uint8_t h_count = 0;        // Overflow tracker for the heading measurement
uint8_t new_heading = 0;    // Flag to denote time to read heading
uint16_t heading = 0;       // Value of heading
uint8_t new_range = 0;      // Flag to denote time to read ranger
uint16_t range = 0;         // Value of range
uint8_t r_count = 0;        // Overflow tracker for ranger measurement
uint8_t new_print = 0;      // Flag to denote time to print
uint8_t p_count = 0;        // Overflow tracker for printing

// Error values
int heading_error, range_error;

// proportional gain constants
float proportional_control_motor = -13.65;
float proportional_control_servo = -1.4995;

// integral gain constant
float integral_constant = -0.013011;

// stores the sum of the errors which is used in the control routine
long error_sum = 0;


int main()
{
    // Initialize all systems
    Sys_Init();
    Port_Init();
    XBR0_Init();
    PCA_Init();
    Interrupt_Init();
    SMB_Init();

    // Do an initial read of the ranger to get start measurement
    ReadRanger();

    // set Motor and Servo Pulsewidth to Neutral
    MOTOR_PW = MOTOR_CENTER;
    PCA0CP1 = 0xFFFF - MOTOR_PW;

    SERVO_PW = SERVO_CENTER;
    PCA0CP0 = 0xFFFF - SERVO_PW;

    // wait 1 second
    PCA_counts = 0;
    while (PCA_counts < 50) Sim_Update();

    // main loop
    while(1){
        Sim_Update();
        if (new_print) { // if it is time to print the measurement
            new_print = 0;
            printf("HEADING: %u \t SERVO_PW: %u\r\n", heading, SERVO_PW); // print statement
        }

        // read the compass at the appropriate time
        if (new_heading) {
            // clear flag
            new_heading = 0;
            // get new heading
            ReadCompass();
            // call driver function
            goNorth();
        }

        // assign both pulsewidths to the CPMs
        Set_Servo_Pulsewidth();
        Set_Motor_Pulsewidth();
    }
}

//-----------------------------------------------------------------------------
// goNorth
//-----------------------------------------------------------------------------
//
// Driver function for the main functionality of this lab. Implements a k-turn
// if necessary, otherwise it uses the Proportional-Integral control equation
// to guide the car towards the North direction
//
void goNorth(void) {
    // desired heading is 0 (NORTH)
    heading_error = 0 - heading;

    // adjust error measurement
    if (heading_error > 1800) {
        heading_error -= 3600;
    } else if (heading_error < -1800) {
        heading_error += 3600;
    }

    // if the error is more than 135 degrees: do a k-turn
    if (heading_error > 1350) {
        // put the car in reverse
        MOTOR_PW = MOTOR_MIN;
        // turn the wheels all the way
        SERVO_PW = SERVO_MAX;
        // do not count the heading error in this iteration
        error_sum -= heading_error;
    }
    // if the error is more than 135 degrees the other way: do a k-turn
    else if (heading_error < -1350) {
        // same functionality as if statement above
        MOTOR_PW = MOTOR_MIN;
        SERVO_PW = SERVO_MIN;
        error_sum -= heading_error;
    }
    // if the error is less than 90 degrees either way, go back to normal driving mode
    else if (heading_error < 900 || heading_error > -900) {
        // add to the error summation
        error_sum += heading_error;
        // put the car in full speed forward
        MOTOR_PW = MOTOR_MAX;
        // set the servo pulsewidth using the proportional control equation
        SERVO_PW = SERVO_CENTER + (proportional_control_servo * heading_error) + (integral_constant * error_sum);
    }
}


//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Set up ports for input and output
//
void Port_Init()
{
    P0MDOUT |= 0x30;  //set output pin for CEX0 or CEX2 in push-pull mode

    // port 3
    P3MDOUT &= 0x1F;
    P3 |= ~0x1F;
}

//-----------------------------------------------------------------------------
// XBR0_Init
//-----------------------------------------------------------------------------
//
// Set up the crossbar
//
void XBR0_Init()
{

    XBR0 |= 0x1D;  //configure crossbar as directed in the laboratory
    XBR0 &= ~0x20;

}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
//
// Set up Programmable Counter Array
//
void PCA_Init(void)
{
    PCA0MD &= 0xF0;  // SYSCLK/12
    PCA0MD |= 0x01;

    // Enable 16-bit PWM CCMs
    PCA0CPM0 = 0xC2;
    PCA0CPM1 = 0xC2;
    PCA0CPM2 = 0xC2;
    PCA0CPM3 = 0xC2;
    PCA0CPM4 = 0xC2;

    // Turn PCA on
    PCA0CN = 0x40;
}

//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Set up the PCA overflow interrupts
//
void Interrupt_Init()
{
    EA = 1;
    EIE1 |= 0x08;
}

void SMB_Init()
{
    // Set the clock rate of the SMBus/I2C and
    // Enable the SMBus/I2C (check manual)
    ENSMB = 1;
    SMB0CR = 0x93;
}

//-----------------------------------------------------------------------------
// ReadCompass
//-----------------------------------------------------------------------------
//
// Read the heading value from the compass
//
void ReadCompass(void){
    i2c_read_data(0xC0, 2, Data, 2);   // Read the 0-3600 heading bytes
    heading = (Data[0] << 8) | Data[1];                  // Put the bytes together
}

//-----------------------------------------------------------------------------
// ReadRanger
//-----------------------------------------------------------------------------
//
// Read the distance value from the ranger and start a ping
//
void ReadRanger(void){
    // Read the first echo from the ranger
    // Put the bytes together and save value to global variable
    // Trigger next measurement
    i2c_read_data(0xE0, 2, Data, 2);
    range = (Data[0] << 8) | Data[1];

    Data[0] = 0x51; // measure in cm
    i2c_write_data(0xE0, 0, Data, 1); // write "measure in cm" command to command register

}

//-----------------------------------------------------------------------------
// Set_Motor_Pulsewidth
//-----------------------------------------------------------------------------
//
// Check boundaries and user input before setting motor pulsewidth value to CPM
//
void Set_Motor_Pulsewidth(void)
{
    // Check Limits
    if (MOTOR_PW > MOTOR_MAX){
        // do something
        MOTOR_PW = MOTOR_MAX;
    } else if (MOTOR_PW < MOTOR_MIN){
        // do something else
        MOTOR_PW = MOTOR_MIN;
    }

    // Check if slide switch is on or not and act if needed
    if (SS1) {
        MOTOR_PW = MOTOR_CENTER;
    }

    // Assign MOTOR_PW to the PCA...
    PCA0CP1 = 0xFFFF - MOTOR_PW;
}

//-----------------------------------------------------------------------------
// Set_Servo_Pulsewidth
//-----------------------------------------------------------------------------
//
// Check boundaries and user input before setting servo pulsewidth values to CPM
//
void Set_Servo_Pulsewidth(void)
{
    //printf("Set servo\r\n");
    // check limits
    if (SERVO_PW > SERVO_MAX) {
        SERVO_PW = SERVO_MAX;
    } else if (SERVO_PW < SERVO_MIN) {
        SERVO_PW = SERVO_MIN;
    }

    // Check slide switch
    if (SS2) {
        SERVO_PW = SERVO_CENTER;
    }

    // Assign SERVO_PW to the PCA
    PCA0CP0 = 0xFFFF - SERVO_PW;
}


//-----------------------------------------------------------------------------
// PCA_ISR
//-----------------------------------------------------------------------------
//
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
//
void PCA_ISR ( void )
{
    if(CF){
        CF = 0;
        PCA0 = 0xFFFF - 36863;
        // increment all counters
        PCA_counts++;
        h_count++;
        r_count++;
        p_count++;

        if(h_count == 2){  // Count 40 ms
            h_count = 0;
            new_heading = 1;
        }
        if (r_count == 5) { // Count 80 ms - set to 5 instead of 4 to prevent wrong measurements due to reading too fast
            r_count = 0;
            new_range = 1;
        }
        if (p_count == 5) { // Count 100 ms
            p_count = 0;
            new_print = 1;
        }
    }
    PCA0CN &= 0x40;
}

// By: Rojan Karn
