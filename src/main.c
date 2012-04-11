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

    // init everything for the LCD
    setupLCD();
    LCDClear(0); // clear the LCD in case it still says something

    // turn on analog input and the H-Bridge
    initAnalogInput(); // in this case B0
    initHBridge();

    // initialize the SPI communication to the dsPIC
    initEncoderSPI();
    getEncoder(TRUE);
    getEncoder(TRUE);  // call twice to set up function to account for rollover
    
    // setup timers
    initCoreTimer();
    initTimer4Interrupt();
 
    // print to the LCD
    sprintf(LCD_Out_Buffer,"Esteban CM");
    LCDWriteString(LCD_Out_Buffer, 1, 1);

    
    // initialize UARTS and interrupts
    NU32_EnableUART1Interrupt();
    NU32_Initialize();
 

    sprintf(NU32_RS232OutBuffer,"Esteban CM\r\n");
    NU32_WriteUART1(NU32_RS232OutBuffer);

    TRISDbits.TRISD13 = 0;
    CS1 = 0;
    setMotorSpeed(MAX_PWM);
    //int test = 1000;
    // infinite loop
    while(1) {
//        if (!NU32USER){
//            test++;
//            SetDCOC4PWM(test);
//            sprintf(LCD_Out_Buffer,"%4d", test);
//            LCDWriteString(LCD_Out_Buffer, 2, 1);
//        }
    }
    return 0;
}

//////ISRs
// Core Timer ISR
void __ISR(_CORE_TIMER_VECTOR, IPL7SRS) CoreTimerISR(void)
{
    // increment time by one every second
    time++;

    // toggle led2 every 1 seconds
    if (time%1 == 0)
        NU32LED2 = !NU32LED2;

    WriteCoreTimer(0);              // set core timer counter to 0
    _CP0_SET_COMPARE(CORE_TICKS);   // must set CP0_COMPARE again after interrupt
    INTClearFlag(_CORE_TIMER_IRQ);  // clear the interrupt flag
}

// Timer4 ISR - for reading encoder at 10Hz
void __ISR(_TIMER_4_VECTOR, ipl3) UpdateEncoder(void)
{
    position = getEncoder(FALSE);
    if (position >= MM_TO_COUNTS(20*283))
        setMotorSpeed(0);
    //int ptON, ptOFF,j = 0;
    
//    CS1 = 0;
//    for (j=0;j<2500;j++){};
//    ptOFF = ReadADC10(0);
//    CS1 = 1;
//    for (j=0;j<2500;j++){};
//    ptON = ReadADC10(0);

    static int count = 0;
    if (count == 62) {
        char message[100];
//        sprintf(LCD_Out_Buffer,"Dist=%5.2f mm", (float) COUNTS_TO_MM(position));
//        LCDWriteString(LCD_Out_Buffer, 2, 1);
//        sprintf(LCD_Out_Buffer,"Dist=%5.2f in", (float) MM_TO_IN(COUNTS_TO_MM(position)));
//        LCDWriteString(LCD_Out_Buffer, 1, 1);
        sprintf(message, "encoder: %ld (counts)\r\n", position);
        NU32_WriteUART1(message);

//        sprintf(LCD_Out_Buffer,"ON=%4d OFF=%4d", ptON,ptOFF);
//        LCDWriteString(LCD_Out_Buffer, 2, 1);
        count = 0;
    }
    count++;


    // clear interrupt flag and exit
    mT4ClearIntFlag();
} // end T4 Interrupt

void __ISR(_UART_1_VECTOR, ipl2) IntUart1Handler(void)
{
    // Is this an RX interrupt?
    if(INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
        char data = UARTGetDataByte(UART1);
        PutCharacter(UART1,data);
        NU32LED1 = !NU32LED1;
        NU32LED2 = !NU32LED2;

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
        if (data == 'g') 
            rotateFanArm(FAN_ARM_VERTICAL);
        if (data == 'c')
            rotateTowerDoor(TOWER_DOOR_CLOSED);
        if (data == 'x')
            rotateTowerDoor(TOWER_DOOR_OPEN);

        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
    }

    // We don't care about TX interrupt
    if(INTGetFlag(INT_SOURCE_UART_TX(UART1))) {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}
