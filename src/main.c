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
    drivingState = STATIONARY;
    numCrates = 0;
    crateInFront = crateInMiddle = 0;
    terminalCounts1 = 0;
    terminalDegrees = 0;

    // set temporary motion control variables
    Kp = 70;
    Kd = 170;
    Ki = 15;
    Kt = 12;
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
    sprintf(LCD_Out_Buffer,"Esteban PCM");
    LCDWriteString(LCD_Out_Buffer, 1, 1);

    // initialize UARTS and interrupts
    NU32_EnableUART1Interrupt();
    NU32_Initialize();

    // print to the computer
    sprintf(NU32_RS232OutBuffer,"Esteban PCM\r\n");
    NU32_WriteUART1(NU32_RS232OutBuffer);

    long blah;
    while(1) {
        if (crateInMiddle == 1 && numCrates < 3) {
            terminalCounts1 = 0;
            //blah = time;
            //while (time < blah+500){};
            blowCubeIn();
        }
        if (crateInMiddle == 1 && numCrates == 3) {
            terminalCounts1 = 0;
            blah = time;
            while (time < blah+200){};
            blowCubeUp();
            blah = time;
            while (time < blah+200){};
            blowCubesOut();
            blah = time;
            while (time < blah+200){};
            blowCubeInAndOut();
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
void __ISR(_TIMER_4_VECTOR, ipl3) Timer4ISR(void)
{
    // initalize all static variables
    static long long oldPos1 = 0;
    static long long oldPos2 = 0;
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
    int reset = 0;

    // if motors are on
    if (drivingState != STATIONARY) {
        // make all encoder readings positive
        if (drivingState == FORWARD) {
            position1 = getEncoder1(FALSE);
            position2 = getEncoder2(FALSE);
        } else if (drivingState == BACKWARD) {
            position1 = -1*getEncoder1(FALSE);
            position2 = -1*getEncoder2(FALSE);
        } else if (drivingState == CCW) {
            position1 = -1*getEncoder1(FALSE);
            position2 = getEncoder2(FALSE);
        } else if (drivingState == CW){                                  
            position1 = getEncoder1(FALSE);
            position2 = -1*getEncoder2(FALSE);
        } else if (drivingState == COLOR_SWITCH) {
            if (lastColor == startingColor) {
                //forward
                position1 = getEncoder1(FALSE);
                position2 = getEncoder2(FALSE);
            } else {
                // backward
                position1 = -1*getEncoder1(FALSE);
                position2 = -1*getEncoder2(FALSE);
            }
        }

        // calculate current velocity of each wheel
        vel1 = (position1 - oldPos1);
        vel2 = (position2 - oldPos2);
        oldPos1 = position1;
        oldPos2 = position2;

        // the tie term keeps track of the difference between the motors
        e_tie += vel1 - vel2;

        // the proportional error term
        e1 += (CPS/TMR4_FREQ) - vel1;
        e2 += (CPS/TMR4_FREQ) - vel2;

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
        update1 = ((Kp*e1) + (Kd*e_dif1) - (Kt*e_tie) + (Ki*e_int1))/K0;      // left motor
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

        //for printing data more slowly to UART
        static int count = 0;
        if (count == 25) {
            //char message[100];
            //sprintf(message, "%ld %lld\r\n", position1, position2);
//            sprintf(message, "K %f %f %f %f vel: %f %f up: %d %d err: %f %f\r\n", Kp, Kd, Ki, vel1, vel2, update1, update2,e1,e2);
//            NU32_WriteUART1(message);
//            sprintf(NU32_RS232OutBuffer,"%d %d\n", (int)(10*vel1), (int)(10*vel2));
//            WriteString(UART1, NU32_RS232OutBuffer);
            count = 0;
        }
        count++;

        // stop the motors if the desired position is reached
        if (drivingState == FORWARD || drivingState == BACKWARD) {
            if (position1 >= terminalCounts1)
                reset = 1;
        } else if (drivingState == CCW) {
            if ((360*(WHEEL_RADIUS/WHEELBASE)*(((float)(position2)-(-1*position1))/COUNTS_PER_REVOLUTION)) >= terminalDegrees)
                reset = 1;
        } else if (drivingState == CW) {
            if ((360*(WHEEL_RADIUS/WHEELBASE)*(((float)(position1)-(-1*position2))/COUNTS_PER_REVOLUTION)) >= terminalDegrees)
                reset = 1;
        } else if (drivingState == COLOR_SWITCH) {
            if (lastColor != currentColor)
                reset = 1;
        }
        
        if (reset != 0) {
            drivingState = STATIONARY;
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
            reset = 0;
        }

        setMotorSpeed(update1, update2);
   
    }



    // clear interrupt flag and exit
    mT4ClearIntFlag();
} // end T4 Interrupt

// Timer5 ISR - for reading phototransisors at 10 Hz
void __ISR(_TIMER_5_VECTOR, ipl3) Timer5ISR(void)
{
    int i, j;
    int ptON[7], ptOFF[7], ptDIF[7];
    int crates = 0;
    long lastTime;
    static int firstTime = 0;
    
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

    // Color sensor 1
    if (ptDIF[0] < CS1_BP_THRESHOLD)
        currentColor1 = BLACK;
    else if (ptDIF[0] < CS1_PW_THRESHOLD)
        currentColor1 = PURPLE;
    else
        currentColor1 = WHITE;

    // Color sensor 2
    if (ptDIF[1] < CS2_BP_THRESHOLD)
        currentColor2 = BLACK;
    else if (ptDIF[1] < CS2_PW_THRESHOLD)
        currentColor2 = PURPLE;
    else
        currentColor2 = WHITE;

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

//    char message[100];
//    sprintf(message, "%d %d %d %d \r\n", startingColor, currentColor, currentColor1, currentColor2);
//    NU32_WriteUART1(message);

    
    // clear interrupt flag and exit
    mT5ClearIntFlag();
} // end T5 Interrupt

void __ISR(_CHANGE_NOTICE_VECTOR, IPL3SOFT) ChangeNotificationISR(void)
{
    PORTB;
    static long lastTime = 0;
    
    if (time > (lastTime+1000)) {
        if (!COLLISION_TOP_LEFT) {
            NU32LED1 = !NU32LED1;
            sprintf(LCD_Out_Buffer,"Top Left    ");
            LCDWriteString(LCD_Out_Buffer, 2, 1);
            turnAngle(-30);
        }
        if (!COLLISION_TOP_RIGHT) {
            NU32LED1 = !NU32LED1;
            sprintf(LCD_Out_Buffer,"Top Right   ");
            LCDWriteString(LCD_Out_Buffer, 2, 1);
            turnAngle(30);
        }
        if (!COLLISION_BOTTOM_LEFT) {
            NU32LED1 = !NU32LED1;
            sprintf(LCD_Out_Buffer,"Bottom Left ");
            LCDWriteString(LCD_Out_Buffer, 2, 1);
            driveDistance(4*11.13);
        }
        if (!COLLISION_BOTTOM_RIGHT) {
            NU32LED1 = !NU32LED1;
            sprintf(LCD_Out_Buffer,"Bottom Right");
            LCDWriteString(LCD_Out_Buffer, 2, 1);
            driveDistance(-4*11.13);
        }
        lastTime = time;
    }    

    mCNClearIntFlag(); // clear the interrupt flag
}

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
        if (data == 'f')
            FAN1 = !FAN1;
        if (data == 'g')
            FAN2 = !FAN2;
        if (data == 's')
            scanLaser();

        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
    }

    // We don't care about TX interrupt
    if(INTGetFlag(INT_SOURCE_UART_TX(UART1))) {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}
