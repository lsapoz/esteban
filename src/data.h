#ifndef __DATA_H_
#define __DATA_H_

// Constants
# define PI 3.14159
#define CORE_TICKS 40000            // 40,000 ticks (1/1000 second)
#define COUNTS_PER_REVOLUTION 8192  // motor's CPR with 4x quadrature
#define MAX_PWM 1250                // 100% duty cycle
#define WHEEL_RADIUS 45             // 45 mm
#define WHEELBASE 174.82            // 174.82 mm
#define COLOR_BASE 150               // 150 mm
#define INERTIA_COUNTS 500          // extra counts recorded after motors stop

// Formulas
#define COUNTS_TO_MM(c) (2*PI*WHEEL_RADIUS*((float)c/COUNTS_PER_REVOLUTION))
#define MM_TO_COUNTS(m) ((float)(COUNTS_PER_REVOLUTION*m)/(2*PI*WHEEL_RADIUS))
#define MM_TO_IN(m) (0.03937*(float)m)
#define IN_TO_MM(i) (25.4*(float)i)
#define POSITION_TO_ANGLE(p,q) (360*(WHEEL_RADIUS/WHEELBASE)*(((float)p+q)/COUNTS_PER_REVOLUTION))
#define RADIANS_TO_DEGREES(r) (r*180/PI)

// Timers
#define TMR4_FREQ 250                           // 250 Hz
#define TMR4_PS 256                             //Prescaler - IF CHANGED, CHANGE IN INITIALIZATION FUNCTION
#define TMR4_PR SYS_FREQ/TMR4_PS/TMR4_FREQ      //Period Register

#define TMR5_FREQ 10                            // 10 Hz
#define TMR5_PS 256                             //Prescaler - IF CHANGED, CHANGE IN INITIALIZATION FUNCTION
#define TMR5_PR SYS_FREQ/TMR5_PS/TMR5_FREQ      //Period Register

// Motion Control
//#define Kp 324
//#define Kd 502
//#define Ki 53
//#define K0 100
#define WINDUP 1000    // windup for the motion control loop

// Driving States
#define STATIONARY 0
#define FORWARD 1
#define BACKWARD 2
#define CCW 3
#define CW 4
#define COLOR_SWITCH 5

// Driving Modes  -  0 is stationary
#define FAST 1
#define MEDIUM 2
#define SLOW 3
#define CORRECTER 4

// Speeds
#define CPS_FAST 8000.0    // 8000 encoder ticks per second
#define CPS_MEDIUM 5000.0
#define CPS_SLOW 3000.0
#define CPS_CORRECTER 1500.0

// Reset States
#define R_NONE 0      // not resetting
#define R_SLOW 1      // begin driving slow
#define R_CORRECTER 2   // begin reversing
#define R_STOP 3     // stop
#define BOUNDARY_COUNTS 3000
#define BOUNDARY_ANGLE 45

// Color States
#define WHITE 0
#define PURPLE 1
#define BLACK 2
#define BOTH 3

// Voltage Thresholds
#define CS1_BP_THRESHOLD 150     // threshold between black and purple for CS1
#define CS1_PW_THRESHOLD 550     // threshold betweeb purple and white for CS1
#define CS2_BP_THRESHOLD 100     // threshold between black and purple for CS2
#define CS2_PW_THRESHOLD 400     // threshold betweeb purple and white for CS2
#define BB_THRESHOLD 100         // break beam threshold
#define LASER_THRESHOLD 100      // threshold for laser

// Fan Arm
#define FAN_ARM_VERTICAL 2950    // "Fan Arm - Vertical" servo pwm
#define FAN_ARM_HORIZONTAL 1615  // "Fan Arm - Horizontal" servo pwm

// Tower Door
#define TOWER_DOOR_OPEN 1935     // "Tower Door - Open" servo pwm
#define TOWER_DOOR_CLOSED 2745   // "Tower Door - Closed" servo pwm
#define TOWER_DOOR_AJAR 2425     // "Tower Door - Ajar" servo pwm

// Laser
#define LASER_LEFT 1168
#define LASER_RIGHT 2568
#define LASER_CENTER 1868
#define LASER_STEP 10
#define LASER_STEP_ANGLE 0.85

// Motor Pins
#define DIR1 LATDbits.LATD5      //motor 1 direction pin - D5
#define DIR2 LATDbits.LATD6      //motor 2 direction pin - D6
#define EN1  LATDbits.LATD7      //motor 1 enable pin - D7
#define EN2  LATDbits.LATD8      //motor 2 enable pin - D8

// Encoder Pins
#define SLAVE_SELECT1 LATFbits.LATF12     // pin F12
#define SLAVE_SELECT2 LATFbits.LATF3      // pin F3

// "Smallest" Analog Pin being read
#define SAP 6   // pin B6 is the "smallest" analog input scanned

// Laser Pins
#define LASER_LIGHT LATDbits.LATD10          // pin D10
#define LASER_PT 6 - SAP                     // pin B6

// LEDs toggle pin
#define LEDS LATDbits.LATD11                 // pin D11

// Fan toggle pins
#define FAN1 LATDbits.LATD12                 // pin D12
#define FAN2 LATDbits.LATD13                 // pin D13

// Color Detection Pins
#define COLOR_SENSOR_LEFT 7 - SAP            // pin B7
#define COLOR_SENSOR_RIGHT 8 - SAP           // pin B8

// Break Beam Pins
#define BB_CHASSIS_FRONT 9 - SAP     // pin B9
#define BB_CHASSIS_MIDDLE 10 - SAP   // pin B10
#define BB_TOWER_BOTTOM 11 - SAP     // pin B11
#define BB_TOWER_MIDDLE 12 - SAP     // pin B12
#define BB_TOWER_TOP 13 - SAP        // pin B13

// Collision Detection Pins
#define COLLISION_TOP_LEFT PORTBbits.RB0   // pin B0 - CN2
#define COLLISION_TOP_RIGHT PORTBbits.RB1  // pin B1 - CN3
#define COLLISION_BOTTOM_LEFT PORTBbits.RB2   // pin B2 - CN4
#define COLLISION_BOTTOM_RIGHT PORTBbits.RB3  // pin B3 - CN5


// Global Variables
long time;          // time in milliseconds
int drivingState, drivingMode;   // one of 5 states
int startingColor;  // "home" color
int currentColor, currentColor1, currentColor2;  // current readings of the color sensors
int lastColor;      // used for driving to center

float cps;

int numCrates;                     // the number of crates in the tower
int crateInFront, crateInMiddle;   // booleans for front and middle break beams

long position1, position2;         // current position of tne encoders
float terminalCounts1;
int terminalDegrees; // these dictate when to stop driving
int globalAngle;

//Motion Control - Temporarily global variables - will become defined constants
float Kp;  // proportional gain for motion control loop
float Ki;  // integral gain for the motion control loop
float Kd;  // differential gain for the motion control loop
float Kt;  // tie gain for the motional control loop
float K0;  // motion divisor used for increasing magnitude of motion control gains
#endif /* __DATA_H_ */
