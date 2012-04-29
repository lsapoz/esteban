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
    drive = 0;
    terminalCounts1 = 0;
    terminalDegrees = 0;
    Kp = 324;
    Kd = 502;
    Ki = 53;
    K0 = 100;

    // init everything for the LCD
    setupLCD();
    LCDClear(0); // clear the LCD in case it still says something

    // turn on analog input and the H-Bridge
    initAnalogInput();
    initHBridge();

    // initialize the SPI communication to the dsPIC
    initEncoderSPI();
    getEncoder1(TRUE);
    getEncoder1(TRUE);  // call twice to set up function to account for rollover
    getEncoder2(TRUE);
    getEncoder2(TRUE);  // call twice to set up function to account for rollover
    
    // setup timers
    initCoreTimer();
    initTimer4Interrupt();
    initTimer5Interrupt();
 
    // initalize digital out pins
    initDigitalOut();

    // initalize change notification
    initChangeNotification();

    // print to the LCD
    sprintf(LCD_Out_Buffer,"Esteban MM");
    LCDWriteString(LCD_Out_Buffer, 1, 1);

    // initialize UARTS and interrupts
    NU32_EnableUART1Interrupt();
    NU32_Initialize();

    // print to the computer
    sprintf(NU32_RS232OutBuffer,"Esteban MM\r\n");
    NU32_WriteUART1(NU32_RS232OutBuffer);

    setMotorSpeed(MAX_PWM,MAX_PWM);
    DIR1 = 0;
    DIR2 = 0;

    //setMotorSpeed(MAX_PWM);
    while(1) {

    }
    return 0;
}

//////ISRs
// Core Timer ISR
void __ISR(_CORE_TIMER_VECTOR, IPL7SRS) CoreTimerISR(void)
{
    // increment time by one every second
    time++;

    // toggle led2 every second
    if (time%100 == 0)
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
    static float e_int = 0;
    static float e_dif1 = 0;
    static float e_dif2 = 0;
    static int update1 = 0;
    static int update2 = 0;
    int reset = 0;

    // if motors are on
    if (drive != 0) {
        // make all encoder readings positive
        if (drive == 1) {                       // forward
            position1 = getEncoder1(FALSE);
            position2 = getEncoder2(FALSE);
        } else if (drive == 2) {                // backward
            position1 = -1*getEncoder1(FALSE);
            position2 = -1*getEncoder2(FALSE);
        } else if (drive == 3) {                // CCW
            position1 = getEncoder1(FALSE);
            position2 = -1*getEncoder2(FALSE);
        } else {                                // CW
            position1 = -1*getEncoder1(FALSE);
            position2 = getEncoder2(FALSE);
        }

        // calculate current velocity of each wheel
        vel1 = (position1 - oldPos1);
        vel2 = (position2 - oldPos2);
        oldPos1 = position1;
        oldPos2 = position2;

        // the integral term keeps track of the difference between the motors
        e_int += vel2 - vel1;

        // the proportional error term
        e1 += (CPS/TMR4_FREQ) - vel1;
        e2 += (CPS/TMR4_FREQ) - vel2;

        // the differentual error term
        e_dif1 = e1 - e1Old;
        e_dif2 = e2 - e2Old;
        e1Old = e1;
        e2Old = e2;

        // calculate the PWM needed to maintain velocity
        update1 = ((Kp*e1) + (Kd*e_dif1) + (Ki*e_int))/K0;      //right motor
        update2 = ((Kp*e2) + (Kd*e_dif2) - (Ki*e_int))/K0;      // left motor

        // do not exceed PWM bounds
        if (update1>MAX_PWM)
            update1 = MAX_PWM;
        if (update1 < 0)
            update1 = 0;
        if (update2>MAX_PWM)
            update2 = MAX_PWM;
        if (update2 < 0)
            update2 = 0;

        // for printing data more slowly to UART
        static int count = 0;
        if (count == 25) {
            char message[100];
            //sprintf(message, "K %f %f %f vel: %f %f up: %d %d err: %f %f\r\n", Kp, Kd, Ki, vel1, vel2, update1, update2,e1,e2);
            sprintf(message, "%ld %lld\r\n", position1, position2);
            NU32_WriteUART1(message);
            count = 0;
        }
        count++;

        // stop the motors if the goal is reached
        if (drive == 1 || drive == 2) {
            if (position1 >= terminalCounts1)
                reset = 1;
        } else if (drive == 3) {
            if ((360*(WHEEL_RADIUS/WHEELBASE)*(((float)(position1)-(-1*position2))/COUNTS_PER_REVOLUTION)) >= terminalDegrees)
                reset = 1;
        } else if (drive == 4) {
            if ((360*(WHEEL_RADIUS/WHEELBASE)*(((float)(position2)-(-1*position1))/COUNTS_PER_REVOLUTION)) >= terminalDegrees)
                reset = 1;
        }
        if (reset != 0) {
            drive = 0;
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
            e_int = 0;
            e_dif1 = 0;
            e_dif2 = 0;
            update1 = 0;
            update2 = 0;
            reset = 0;
        }

        setMotorSpeed(update2, update1);
            
    }



    // clear interrupt flag and exit
    mT4ClearIntFlag();
} // end T4 Interrupt

// Timer5 ISR - for reading phototransisors at 10 Hz
void __ISR(_TIMER_5_VECTOR, ipl3) Timer5ISR(void)
{
    //Kp = ReadADC10(0);
    //Kd = ReadADC10(1);
    //Ki = ReadADC10(2);

    int ptON, ptOFF,j = 0;

    COLOR_SENSOR1 = 1;
    for (j=0;j<5000;j++){};
    ptON = ReadADC10(3);
    COLOR_SENSOR1 = 0;
    for (j=0;j<5000;j++){};
    ptOFF = ReadADC10(3);


    // clear interrupt flag and exit
    mT5ClearIntFlag();
} // end T5 Interrupt

void __ISR(_CHANGE_NOTICE_VECTOR, IPL3SOFT) ChangeNotificationISR(void)
{
    PORTD;
    static int trig = 0;
    static int dir = 1;
    
    if (time > (trig+100)) {
        if (!COLLISION1) {
            NU32LED1 = !NU32LED1;
//            driveDistance(dir,2*11.13);
//            dir = 3 - dir;
            //turnAngle(dir,90);
            //dir = 7 - dir;
        }
        trig = time;
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

        // d = reverse direction
        if (data == 'd')
            reverseDirection();

        // e = toggle enable pins
        if (data == 'e') {
            EN1 = !EN1;
            EN2 = !EN2;
        }

//        // s = sweep servo
//        if (data == 's') {
//            int i = 0;
//            int delay = 0;
//
//            // move the servo from 850 to 2900 (30 to 120 degrees)
//            // read the phototransistor on pin B1 at each step and save it in a buffer
//            for (i=0; i<1500; i++) {
//                SetDCOC3PWM(i+750);
//                for (delay=0; delay<3000; delay++) {} // wait for the rc servo to move
//            }
//
//            // move the servo back to 30 degrees
//            SetDCOC3PWM(750);
//        }
        if (data == 'h')
            rotateFanArm(FAN_ARM_HORIZONTAL);
        if (data == 'v')
            rotateFanArm(FAN_ARM_VERTICAL);
        if (data == 'c')
            rotateTowerDoor(TOWER_DOOR_CLOSED);
        if (data == 'o')
            rotateTowerDoor(TOWER_DOOR_OPEN);

        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
    }

    // We don't care about TX interrupt
    if(INTGetFlag(INT_SOURCE_UART_TX(UART1))) {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}
