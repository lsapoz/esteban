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
int sweepLaser(float feet);

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

void driveToCenter();
void driveToZone();

void resetAngle();
void resetAngleOnWall(int wall);
void resetAngleInZone();
void exitAndResetOnRight();
void exitAndResetOnLeft();

// Sweep Patterns
void firstSweepPattern1();  // first in front, two on middle line, one on first line
void firstSweepPattern2();  // first in front, three on second line
void firstSweepPattern3();  // the four diaganals
void firstSweepPattern4();  // facing on the wall
void firstSweepPattern5();  // first four THIS ONE

void secondSweepPattern1();
void secondSweepPattern2(); // THIS ONE
void secondSweepPattern3(); // AND THIS ONE

void randomSweepPattern();
void wallToWallSweepPattern();

// Stringing it all together
void firstSweep();
void secondSweep();
void thirdSweep();
void randomSweep();

#endif /* __LIBRARY_H_ */
