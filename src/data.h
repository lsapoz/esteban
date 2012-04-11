#ifndef __DATA_H_
#define __DATA_H_

// Constants
#define CORE_TICKS 40000000  // 40 M ticks (one second)
#define COUNTS_PER_REVOLUTION 8192 // motor's CPR with 4x quadrature
#define MAX_PWM 1250
#define WHEEL_RADIUS 45

// Formulas
#define COUNTS_TO_MM(c) (2*3.14159*WHEEL_RADIUS*((float)c/COUNTS_PER_REVOLUTION))
#define MM_TO_COUNTS(m) ((float)(COUNTS_PER_REVOLUTION*m)/(2*3.14159*WHEEL_RADIUS))
#define MM_TO_IN(m) (0.03937*(float)m)

// Pins
#define DIR1 LATDbits.LATD5  //motor 1 direction pin - D5
#define DIR2 LATDbits.LATD6  //motor 2 direction pin - D6
#define EN1  LATDbits.LATD7  //motor 1 enable pin - D7
#define EN2  LATDbits.LATD8  //motor 2 enable pin - D8
#define CS1 LATDbits.LATD13  // color sensor 1 LED - D13

// Fan Arm
#define FAN_ARM_VERTICAL 580      // "Fan Arm - Vertical" servo pwm
#define FAN_ARM_HORIZONTAL 1650   // "Fan Arm - Horizontal" servo pwm

// Tower Door
#define TOWER_DOOR_OPEN 2150     // "Tower Door - Open" servo pwm
#define TOWER_DOOR_CLOSED 2840   // "Tower Door - Closed" servo pwm


// Global Variables
int time;
long long position;
#endif /* __DATA_H_ */
