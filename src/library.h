#ifndef __LIBRARY_H__
#define __LIBRARY_H__

// Timer functions
void initCoreTimer(void);
void initTimer4Interrupt(void);

// ADC functions
void initAnalogInput(void);

// Encoder functions
void initEncoderSPI(void);
long long getEncoder(int reset);

// Motor functions
void initHBridge(void);
void setMotorSpeed(int dutyCycle);
void reverseDirection();

// Fan Arm Functions
void rotateFanArm(int servoPos);

// Door Functions
void rotateTowerDoor(int servoPos);
#endif /* __LIBRARY_H_ */
