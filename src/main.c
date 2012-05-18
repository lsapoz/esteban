#include <plib.h>
#include "data.h"
#include "library.h"
#include "NU32.h"
#include "LCD.h"
#include "math.h"

//////MAIN
int main(void)
{
    // setup LED's and buttons
    NU32_Startup();

    // initalize global variables
    time = 0;
    goBack = 0;
    drivingState = STATIONARY;
    drivingMode = STATIONARY;
    numCrates = 0;
    crateInFront = 0;
    crateInMiddle = 0;
    terminalCounts1 = 0;
    terminalDegrees = 0;
    globalAngle = 0;
    cubesDeposited = 0;
    sweepRound = 1;

    collisionBottomLeft = 0;
    collisionBottomRight = 0;
    collisionTopLeft = 0;
    collisionTopRight = 0;

    // set temporary motion control variables
    Kp = 30;
    Kd = 120;
    Ki = 5;
    Kt = 10;
    K0 = 100;

    // init everything for the LCD
    setupLCD();
    LCDClear(0); // clear the LCD in case it still says something

    // initalize pins
    initChangeNotification();
    initDigitalOut();
    initAnalogInput();
    initOutputCompare();

    // initialize the SPI communication to the dsPIC
    initEncoderSPI();
    getEncoder1(TRUE);getEncoder1(TRUE); // call twice to set up function to account for rollover
    getEncoder2(TRUE);getEncoder2(TRUE);  // call twice to set up function to account for rollover
    
    // setup timers
    initCoreTimer();
    initTimer4Interrupt();
    initTimer5Interrupt();

    // print to the LCD
    sprintf(LCD_Out_Buffer,"Esteban DC");
    LCDWriteString(LCD_Out_Buffer, 1, 1);

    // initialize UARTS and interrupts
    NU32_EnableUART1Interrupt();
    NU32_Initialize();

    // print to the computer
    sprintf(NU32_RS232OutBuffer,"Esteban DC\r\n");
    NU32_WriteUART1(NU32_RS232OutBuffer);

    long blah = time;
    while(1) {
//        if (crateInMiddle == 1 && numCrates < 3) {
//            blah = time;
//            while (time < blah + 500){};
//            blowCubeIn();
//        }
//        if (crateInMiddle == 1 && numCrates == 3) {
//            blah = time;
//            while (time < blah+1000){};
//            blowCubeUp();
//            blah = time;
//            while (time < blah+500){};
//            blowCubesOut();
//            blah = time;
//            while (time < blah+500){};
//            blowCubeInAndOut();
//        }

        if (!NU32USER) {
            //driveDistance(24,PLAID);
            //turnAngle(90);
            firstSweep2();
            while (drivingState != STATIONARY){};
            resetAngle();
            while (drivingState != STATIONARY){};
            driveToCenter();
            while (drivingState != STATIONARY){};
            driveToZone();
            while (drivingState != STATIONARY){};
            driveDistance(24,MEDIUM);
            while (drivingState != STATIONARY){};
            turnAngle(70);
            while (drivingState != STATIONARY){};
            resetAngleOnWall(RIGHT);
            while (drivingState != STATIONARY){};
            driveDistance(-6,PLAID);
            while (drivingState != STATIONARY){};
            resetAngle();
        }
}
    return 0;
}

//////ISRs
// Core Timer ISR
void __ISR(_CORE_TIMER_VECTOR, IPL7SRS) CoreTimerISR(void)
{
    // increment time by one every millisecond
    time++;

    // toggle led2 every second
    if (time%1000 == 0)
        NU32LED2 = !NU32LED2;

    WriteCoreTimer(0);              // set core timer counter to 0
    _CP0_SET_COMPARE(CORE_TICKS);   // must set CP0_COMPARE again after interrupt
    INTClearFlag(_CORE_TIMER_IRQ);  // clear the interrupt flag
}

// Timer4 ISR - for reading encoder at 250Hz
void __ISR(_TIMER_4_VECTOR, ipl5) Timer4ISR(void)
{
    // initalize all static variables
    static long oldPos1 = 0;
    static long oldPos2 = 0;
    static float vel1 = 0;
    static float vel2 = 0;
    static float e1 = 0;
    static float e2 = 0;
    static float e1Old = 0;
    static float e2Old = 0;
    static float e_int1 = 0;
    static float e_int2 = 0;
    static float e_dif1 = 0;
    static float e_dif2 = 0;
    static float e_tie = 0;
    static int update1 = 0;
    static int update2 = 0;

    static long overshootCounts = 0;
    static float stoppedDegrees = 0;
    static float overshootDegrees = 0;

    static long waitTime = 0;

    static float Kpadj = 0;
    static float Kdadj = 0;
    static float Kiadj = 0;
    
    int reset = R_NONE;

//    position1 = getEncoder1(FALSE);
//    position2 = getEncoder2(FALSE);
//    static int count = 0;
//    if (count == 50) {
//            sprintf(LCD_Out_Buffer, "%3.4f", POSITION_TO_ANGLE(position1, -position2));
//            LCDWriteString(LCD_Out_Buffer, 2, 1);
//            count = 0;
//        }
//    count++;

    // if motors are on
    if (drivingState != STATIONARY) {
        // make all encoder readings positive
        if (drivingState == FORWARD || drivingState == RESET_ANGLE_WALL) {
            Kpadj = 2;
            Kt = 1;
            position1 = getEncoder1(FALSE);
            position2 = getEncoder2(FALSE);
        } else if (drivingState == BACKWARD || drivingState == RESET_ANGLE_ZONE) {
            Kp = 26;
            Kd = 115;
            Kpadj = 6;
            Kdadj = 7;
            Kt = 1;
            position1 = -getEncoder1(FALSE);
            position2 = -1.01*getEncoder2(FALSE);
        } else if (drivingState == CCW) {
            Kpadj = 3;
            Kdadj = 7;
            Kp = 45;
            Kd = 180;
            Ki = 8;
            Kt = 1;
            position1 = -getEncoder1(FALSE);
            position2 = getEncoder2(FALSE);
        } else if (drivingState == CW){
            Kpadj = 2;
            Kp = 45;
            Kd = 180;
            Ki = 8;
            Kt = 1;
            position1 = getEncoder1(FALSE);
            position2 = -getEncoder2(FALSE);
        } else if (drivingState == COLOR_SWITCH) {
            if (lastColor == startingColor) {
                Kpadj = 2;
                Kt = 1;
                position1 = getEncoder1(FALSE);
                position2 = getEncoder2(FALSE);
            } else {
                Kp = 26;
                Kd = 115;
                Kpadj = 6;
                Kdadj = 7;
                Kt = 1;
                position1 = -getEncoder1(FALSE);
                position2 = -1.01*getEncoder2(FALSE);
            }
        }

        if (drivingMode == STATIONARY) {
            cps = 0;
        } else if (drivingMode == FAST) {
            cps = CPS_FAST;
        } else if (drivingMode == MEDIUM) {
            cps = CPS_MEDIUM;
        } else if (drivingMode == SLOW) {
            cps = CPS_SLOW;
        } else if (drivingMode == CORRECTER) {
            cps = CPS_CORRECTER;
        } else if (drivingMode == PLAID) {
            cps = CPS_PLAID;
        }

        // calculate current velocity of each wheel
        vel1 = (position1 - oldPos1);
        vel2 = (position2 - oldPos2);
        oldPos1 = position1;
        oldPos2 = position2;

        // the tie term keeps track of the difference between the motors
        e_tie = position1 - position2;


        // the proportional error term
        e1 += (cps/TMR4_FREQ) - vel1;
        e2 += (cps/TMR4_FREQ) - vel2;

        // the differentual error term
        e_dif1 = e1 - e1Old;
        e_dif2 = e2 - e2Old;
        e1Old = e1;
        e2Old = e2;

        // the integral term keeps track of the buildup of error
        e_int1 += e1;
        e_int2 += e2;

        // do not exceed windup
        if (e_int1 > WINDUP)
            e_int1 = WINDUP;
        if (e_int1 < -WINDUP)
            e_int1 = -WINDUP;
        if (e_int2 > WINDUP)
            e_int2 = WINDUP;
        if (e_int2 < -WINDUP)
            e_int2 = -WINDUP;

        // calculate the PWM needed to maintain velocity
        update1 = (((Kp+Kpadj)*e1) + ((Kd+Kdadj)*e_dif1) - (Kt*e_tie) + ((Ki+Kiadj)*e_int1))/K0;      // left motor
        update2 = ((Kp*e2) + (Kd*e_dif2) + (Kt*e_tie) + (Ki*e_int2))/K0;      // right motor

        // do not exceed PWM bounds
        if (update1 > MAX_PWM)
            update1 = MAX_PWM;
        if (update1 < 0)
            update1 = 0;
        if (update2 > MAX_PWM)
            update2 = MAX_PWM;
        if (update2 < 0)
            update2 = 0;

//        //for printing data more slowly to UART
//        static int count = 0;
//        if (count == 25) {
//            char message[100];
//            //sprintf(message, "%ld %lld\r\n", position1, position2);
//            sprintf(message, "vel: %f %f up: %d %d err: %f %f\r\n", vel1, vel2, update1,update2,e1,e2);
//            NU32_WriteUART1(message);
////            sprintf(NU32_RS232OutBuffer,"%d %d\n", (int)(10*vel1), (int)(10*vel2));
////            WriteString(UART1, NU32_RS232OutBuffer);
//            count = 0;
//        }
//        count++;

        // stop the motors if the desired position is reached
        if (drivingMode == PLAID || drivingMode == FAST || drivingMode == MEDIUM) {
            if (drivingState == FORWARD || drivingState == BACKWARD) {
                if (position1 >= terminalCounts1)
                    reset = R_STOP;
            } else if (drivingState == CCW || drivingState == CW) {
                if (POSITION_TO_ANGLE(position1,position2) >= terminalDegrees - BOUNDARY_ANGLE)
                    reset = R_SLOW;
            } else if (drivingState == COLOR_SWITCH) {
                if (lastColor != currentColor1 && lastColor != currentColor2)
                    reset = R_STOP;
            } else if (drivingState == RESET_ANGLE_WALL) {
                if (collisionTopLeft == 1 && EN1 == 1) {
                    EN1 = 0;
                    Kt = 0;
                    waitTime = time;
                    drivingMode = FAST;
                }
                if (collisionTopRight == 1 && EN2 == 1) {
                    EN2 = 0;
                    Kt = 0;
                    waitTime = time;
                    drivingMode = FAST;
                }
                if (EN1 == 0 && EN2 == 0) {
                     reset = R_STOP;
                     EN1 = 1;
                     EN2 = 1;
                }
                if (time > waitTime + 750 && (EN1==0 || EN2==0))
                    reset = R_STOP;
            } else if (drivingState == RESET_ANGLE_ZONE) {
                if (collisionBottomLeft == 1 && EN1 == 1) {
                    EN1 = 0;
                    Kt = 0;
                    waitTime = time;
                    drivingMode = FAST;
                }
                if (collisionBottomRight == 1 && EN2 == 1) {
                    EN2 = 0;
                    Kt = 0;
                    waitTime = time;
                    drivingMode = FAST;
                }
                if (EN1 == 0 && EN2 == 0) {
                     reset = R_STOP;
                     EN1 = 1;
                     EN2 = 1;
                }
                if (time > waitTime + 750 && (EN1==0 || EN2==0))
                    reset = R_STOP;
            }
        } else if (drivingMode == SLOW) {
            if (drivingState == FORWARD || drivingState == BACKWARD) {
                if (position1 >= terminalCounts1)
                    reset = R_STOP;
            } else if (drivingState == CCW || drivingState == CW) {
                if (POSITION_TO_ANGLE(position1,position2) >= terminalDegrees) {
                    reset = R_CORRECTER;
                }  
            }       
        } else if (drivingMode == CORRECTER) {
            if (drivingState == FORWARD || drivingState == BACKWARD) {
                if (position1 >= overshootCounts)
                    reset = R_STOP;
            } else if (drivingState == CCW || drivingState == CW) {
                if (POSITION_TO_ANGLE(position1,position2) >= overshootDegrees)
                    reset = R_STOP;
            }
        }
    }

    if (reset == R_STOP) {
        drivingState = STATIONARY;
        drivingMode = STATIONARY;
        Kp = 30;
        Kd = 120;
        Ki = 5;
        Kt = 10;
        Kpadj = 0;
        Kdadj = 0;
        Kiadj = 0;
        terminalCounts1 = 0;
        terminalDegrees = 0;
        oldPos1 = 0;
        oldPos2 = 0;
        vel1 = 0;
        vel2 = 0;
        e1 = 0;
        e2 = 0;
        e1Old = 0;
        e2Old = 0;
        e_int1 = 0;
        e_int2 = 0;
        e_dif1 = 0;
        e_dif2 = 0;
        e_tie = 0;
        update1 = 0;
        update2 = 0;
        stoppedDegrees = 0;
        overshootCounts = 0;
        overshootDegrees = 0;
        reset = R_NONE;
    } else if (reset == R_SLOW) {
        drivingMode = SLOW;
        reset = R_NONE;
    } else if (reset == R_CORRECTER) {
        // set cps to 0
        drivingMode = STATIONARY;

        // Stop the motors
        setMotorSpeed(0, 0);

        stoppedDegrees = POSITION_TO_ANGLE(position1,position2);

        // determine the inertial overshoot
        waitTime = time;
        while (time < waitTime + 100 + min(terminalDegrees,100)){};   // wait 1/10 second
        getEncoder1(FALSE);
        getEncoder2(FALSE);
        if (drivingState == FORWARD) {
            position1 = getEncoder1(FALSE);
            position2 = getEncoder2(FALSE);
        } else if (drivingState == BACKWARD) {
            position1 = -getEncoder1(FALSE);
            position2 = -getEncoder2(FALSE);
        } else if (drivingState == CCW) {
            position1 = -getEncoder1(FALSE);
            position2 = getEncoder2(FALSE);
        } else if (drivingState == CW) {
            position1 = getEncoder1(FALSE);
            position2 = -getEncoder2(FALSE);
        }
        overshootCounts = position1 - terminalCounts1;
        overshootDegrees = (POSITION_TO_ANGLE(position1, position2) - stoppedDegrees)/min(max((terminalDegrees/150+1.45),1.65),2.00);

        // change direction
        if (drivingState == FORWARD)
            drivingState = BACKWARD;
        else if (drivingState == BACKWARD)
            drivingState = FORWARD;
        else if (drivingState == CW)
            drivingState = CCW;
        else if (drivingState == CCW)
            drivingState = CW;
        DIR1 = !DIR1;
        DIR2 = !DIR2;

        // reset everything
        getEncoder1(TRUE);getEncoder2(TRUE);
        getEncoder1(TRUE);getEncoder2(TRUE);
        oldPos1 = 0;
        oldPos2 = 0;
        vel1 = 0;
        vel2 = 0;
        e1 = 0;
        e2 = 0;
        e1Old = 0;
        e2Old = 0;
        e_int1 = 0;
        e_int2 = 0;
        e_dif1 = 0;
        e_dif2 = 0;
        e_tie = 0;
        update1 = 0;
        update2 = 0;
        terminalCounts1 = 0;
        terminalDegrees = 0;

        // set driving mode to inertial correcter
        drivingMode = CORRECTER;
    } 

    setMotorSpeed(update1, update2);

    // clear interrupt flag and exit
    mT4ClearIntFlag();
} // end T4 Interrupt

// Timer5 ISR - for reading phototransisors at 10 Hz
void __ISR(_TIMER_5_VECTOR, ipl4) Timer5ISR(void)
{
    int i, j;
    //static int count = 0;
    //count++;
    int ptON[7], ptOFF[7], ptDIF[7];
    int collision[4];
    int crates = 0;
    long lastTime;
    static int firstTime = 0;
    int tempColor1 = 0;
    int tempColor2 = 0;
    static int previousColor1 = 0;
    static int previousColor2 = 0;
    
    LEDS = 1;
    lastTime = time;
    while (time<lastTime+2){}; // wait 2 ms
    for (i=COLOR_SENSOR_LEFT, j=0; i<=BB_TOWER_TOP; i++,j++)
        ptON[j] = ReadADC10(i);

    LEDS = 0;
    lastTime = time;
    while (time<lastTime+2){}; // wait 2 ms
    for (i=COLOR_SENSOR_LEFT, j=0; i<=BB_TOWER_TOP; i++,j++)
        ptOFF[j] = ReadADC10(i);

    for (j=0; j<7; j++)
        ptDIF[j] = ptON[j]-ptOFF[j];

    //if ((drivingState != STATIONARY
    // Color sensor 1
    if (ptDIF[0] < CS1_BP_THRESHOLD)
        tempColor1 = BLACK;
    else if (ptDIF[0] < CS1_PW_THRESHOLD )
        tempColor1 = PURPLE;
    else
        tempColor1 = WHITE;

    // Color sensor 2
    if (ptDIF[1] < CS2_BP_THRESHOLD)
        tempColor2 = BLACK;
    else if (ptDIF[1] < CS2_PW_THRESHOLD)
        tempColor2 = PURPLE;
    else
        tempColor2 = WHITE;

    if (firstTime == 0) {
        previousColor1 = tempColor1;
        previousColor2 = tempColor2;
    }

    // Color sensor 1
    if (tempColor1 == previousColor1) {
        if (tempColor1 == BLACK)
            currentColor1 = BLACK;
        else if (tempColor1 == PURPLE)
            currentColor1 = PURPLE;
        else
            currentColor1 = WHITE;
    }

    // Color sensor 2
    if (tempColor2 == previousColor2) {
        if (tempColor2 == BLACK)
            currentColor2 = BLACK;
        else if (tempColor2 == PURPLE)
            currentColor2 = PURPLE;
        else
            currentColor2 = WHITE;
    }

    previousColor1 = tempColor1;
    previousColor2 = tempColor2;

    // If first time, set starting color
    if (firstTime == 0) {
        if (currentColor1 == currentColor2 && currentColor1 != BLACK) {
            startingColor = currentColor1;
            firstTime++;    // never run this loop again
        }
    }

    if (currentColor1 != BLACK && currentColor2 != BLACK) {
        if (currentColor1 == currentColor2)
            currentColor = currentColor1;
        else
            currentColor = BOTH;
    }

    // breakbeam - chassis - front
    if (ptDIF[2] < BB_THRESHOLD)
        crateInFront = 1;
    else
        crateInFront = 0;

    // breakbeam - chassis - middle
    if (ptDIF[3] < BB_THRESHOLD)
        crateInMiddle = 1;
    else
        crateInMiddle = 0;

    // breakbeam - chassis - middle
    if (ptDIF[4] < BB_THRESHOLD)
        crates++;
    if (ptDIF[5] < BB_THRESHOLD)
        crates++;
    if (ptDIF[6] < BB_THRESHOLD)
        crates++;
    numCrates = crates;

    // collision detection
    for (i=COLLISION_TOP_LEFT, j=0; i<=COLLISION_BOTTOM_RIGHT; i++,j++)
        collision[j] = ReadADC10(i);

    if (collision[0] < COLLISION_THRESHOLD)
        collisionTopLeft = 1;
    else
        collisionTopLeft = 0;

    if (collision[1] < COLLISION_THRESHOLD) 
        collisionTopRight = 1;
    else
        collisionTopRight = 0;
    
    if (collision[2] < COLLISION_THRESHOLD) 
        collisionBottomLeft = 1;
    else
        collisionBottomLeft = 0;
    
    if (collision[3] < COLLISION_THRESHOLD) 
        collisionBottomRight = 1;
    else
        collisionBottomRight = 0;

//    int laseron, laseroff, laser;
//    LASER_LIGHT = 1;
//    lastTime = time;
//    while (time<lastTime+3){}; // wait 2 ms
//    laseron = ReadADC10(LASER_PT);
//
//    LASER_LIGHT = 0;
//    lastTime = time;
//    while (time<lastTime+3){}; // wait 2 ms
//    laseroff = ReadADC10(LASER_PT);
//
//    char message[100];
//    sprintf(message, "%d\r\n", laseron-laseroff);
//    NU32_WriteUART1(message);

//    char message[100];
//    sprintf(message, "%d %d %d %d \r\n", collisionTopLeft, collisionTopRight, collisionBottomLeft, collisionBottomRight);
//    NU32_WriteUART1(message);

//    char message[100];
//    sprintf(message, "%d %d %d %d %d %d %d\r\n", ptDIF[0], ptDIF[1], ptDIF[2], ptDIF[3], ptDIF[4], ptDIF[5], ptDIF[6]);
//    NU32_WriteUART1(message);

//    char message[100];
//    sprintf(message, "%d %d %d %d %d %d\r\n", startingColor, currentColor, currentColor1, currentColor2, ptDIF[0], ptDIF[1]);
//    NU32_WriteUART1(message);

    // clear interrupt flag and exit
    mT5ClearIntFlag();
} // end T5 Interrupt

void __ISR(_UART_1_VECTOR, ipl2) IntUart1Handler(void)
{
    // Is this an RX interrupt?
    if(INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
        char data = UARTGetDataByte(UART1);
        PutCharacter(UART1,data);
        NU32LED1 = !NU32LED1;

        if (data == 'h')
            rotateFanArm(FAN_ARM_HORIZONTAL);
        if (data == 'v')
            rotateFanArm(FAN_ARM_VERTICAL);
        if (data == 'c')
            rotateTowerDoor(TOWER_DOOR_CLOSED);
        if (data == 'o')
            rotateTowerDoor(TOWER_DOOR_OPEN);
        if (data == 'a')
            rotateTowerDoor(TOWER_DOOR_AJAR);
        if (data == 'l')
            SetDCOC5PWM(LASER_LEFT);
        if (data == 'r')
            SetDCOC5PWM(LASER_RIGHT);
        if (data == 'k')
            SetDCOC5PWM(LASER_CENTER);
        if (data == 'f')
            FAN1 = !FAN1;
        if (data == 'g')
            FAN2 = !FAN2;
        if (data == 's')
            sweepLaser(3);

        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
    }

    // We don't care about TX interrupt
    if(INTGetFlag(INT_SOURCE_UART_TX(UART1))) {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}
