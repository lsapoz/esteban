#ifndef __LIBRARY_H__
#define __LIBRARY_H__

// Timer functions
void initCoreTimer(void);
void initTimer4Interrupt(void);
void initTimer5Interrupt(void);

// Pin Initalization functions
void initChangeNotification(void);
void initDigitalOut(void);
void initAnalogInput(void);
void initOutputCompare(void);

// Encoder functions
void initEncoderSPI(void);
long getEncoder1(int reset);
long getEncoder2(int reset);

// Servo functions
void rotateFanArm(int servoPos);
void rotateTowerDoor(int servoPos);
int sweepLaser(int feet);

// Fan functions
void blowCubeIn();
void blowCubeUp();
void blowCubesOut();
void blowCubeInAndOut();
void checkForCubes();

// Driving Functions
void setMotorSpeed(int dutyCycle1, int dutyCycle2);
void reverseDirection();
void driveDistance(float inches, int mode);
void turnAngle(int degrees);
void resetAngle();
void driveToCenter();
void resetAngleOnWall(int wall);    // STILL NEED TO ACTUALLY RESET GLOBAL ANGLE
void resetAngleInZone();
void driveToZone();

// Strategy Functions
void firstSweep1();
void firstSweep2();
void firstSweep3();
#endif /* __LIBRARY_H_ */
