#ifndef __DATA_H_
#define __DATA_H_

// Constants
#define CORE_TICKS 400000  // 400 K ticks (1/100 second)
#define COUNTS_PER_REVOLUTION 8192 // motor's CPR with 4x quadrature
#define MAX_PWM 1250
#define WHEEL_RADIUS 45     // 45 mm
#define WHEELBASE 174.82    // 174.82 mm
#define INERTIA_COUNTS 500
#define CPS 4000.0    // 4000 encoder ticks per second
#define WINDUP 200 // windup for the motion control loop

//#define Kp 324
//#define Kd 502
//#define Ki 53
//#define K0 100

// Formulas
#define COUNTS_TO_MM(c) (2*3.14159*WHEEL_RADIUS*((float)c/COUNTS_PER_REVOLUTION))
#define MM_TO_COUNTS(m) ((float)(COUNTS_PER_REVOLUTION*m)/(2*3.14159*WHEEL_RADIUS))
#define MM_TO_IN(m) (0.03937*(float)m)
#define IN_TO_MM(i) (25.4*(float)i)

// Timers
#define TMR4_FREQ 250                           // 250 Hz
#define TMR4_PS 256                             //Prescaler - IF CHANGED, CHANGE IN APPRORIPATE FUNCTION
#define TMR4_PR SYS_FREQ/TMR4_PS/TMR4_FREQ      //Period Register

#define TMR5_FREQ 10                            // 10 Hz
#define TMR5_PS 256                             //Prescaler - IF CHANGED, CHANGE IN APPRORIPATE FUNCTION
#define TMR5_PR SYS_FREQ/TMR5_PS/TMR5_FREQ      //Period Register

// Motor Pins
#define DIR1 LATDbits.LATD5  //motor 1 direction pin - D5
#define DIR2 LATDbits.LATD6  //motor 2 direction pin - D6
#define EN1  LATDbits.LATD7  //motor 1 enable pin - D7
#define EN2  LATDbits.LATD8  //motor 2 enable pin - D8

//// Encoder Pins
#define SLAVE_SELECT1 LATFbits.LATF12   // pin F12
#define SLAVE_SELECT2 LATFbits.LATF3    // pin F3

// Color Detection Pins
#define COLOR_SENSOR1 LATDbits.LATD12  // color sensor 1 LED - D12

// Break Beam Pins
#define BREAK_BEAM_PT_3 3       // lower tower break beam ADC pin

// Collision Detection Pins
#define COLLISION1 PORTDbits.RD13   // D13

// Fan Arm
#define FAN_ARM_VERTICAL 2895      // "Fan Arm - Vertical" servo pwm
#define FAN_ARM_HORIZONTAL 1750   // "Fan Arm - Horizontal" servo pwm

// Tower Door
#define TOWER_DOOR_OPEN 1925     // "Tower Door - Open" servo pwm
#define TOWER_DOOR_CLOSED 2745   // "Tower Door - Closed" servo pwm


// Global Variables
int time;
long long position1, position2;
int drive;  // 0 = still, 1 = Forward, 2 = Backward, 3 = CCW, 4 = CW
float terminalCounts1, terminalDegrees;

//Motion Control
float Kp;  // proportional gain for motion control loop
float Ki;  // integral gain for the motion control loop
float Kd;  // differential gain for the motion control loop
float K0;  // motion divisor used for increasing magnitude of motion control gains
#endif /* __DATA_H_ */
