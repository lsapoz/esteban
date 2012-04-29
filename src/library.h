#ifndef __LIBRARY_H__
#define __LIBRARY_H__

// Timer functions
void initCoreTimer(void);
void initTimer4Interrupt(void);
void initTimer5Interrupt(void);

// ADC functions
void initAnalogInput(void);

// Digital IO functions
void initDigitalOut(void);
void initChangeNotification(void);

// Encoder functions
void initEncoderSPI(void);
long long getEncoder1(int reset);
long long getEncoder2(int reset);

// Motor functions
void initHBridge(void);
void setMotorSpeed(int dC1, int dC2);
void reverseDirection();

// Fan Arm Functions
void rotateFanArm(int servoPos);

// Door Functions
void rotateTowerDoor(int servoPos);

// Driving Functions
void driveDistance(int dir, float inches);
void turnAngle(int dir, float degrees);
#endif /* __LIBRARY_H_ */
