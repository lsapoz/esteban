//////LIBRARIES
#include <plib.h> // for PIC specific functions
#include "NU32.h" // for NU32 board specific functions
#include "LCD.h" // for the 16x2 LCD

//////CONSTANTS
#define CORE_TICKS 10000000  // 10 M ticks (.25 seconds)
#define MAX_PWM 1250

//////PINS
#define DIR1 LATDbits.LATD5  //motor 1 direction pin - D5
#define DIR2 LATDbits.LATD6  //motor 2 direction pin - D6
#define EN1  LATDbits.LATD7  //motor 1 enable pin - D7
#define EN2  LATDbits.LATD8  //motor 2 enable pin - D8

//////GLOBAL VARIABLES
int time = 0;

//////FUNCTION PROTOTYPES
void initCoreTimer(void);
void initAnalogInput(void);
void initHBridge(void);
int getDutyCycle(void);
int getPTValue(void);
void setMotorSpeed(int dutyCycle);
void reverseDirection(void);

//////MAIN
int main(void)
{
    /* setup LED's and buttons: */
    NU32_Startup();
    
    // init everything for the LCD
    setupLCD();
    LCDClear(0); // clear the LCD in case it still says something    

    /* setup core timer*/
    initCoreTimer();
    
    // turn on analog input and the H-Bridge
    initAnalogInput(); // in this case B0
    initHBridge();

    /* initialize UARTS and interrupts */
    NU32_Initialize();
    NU32_EnableUART1Interrupt();
    
    // print to the LCD
    sprintf(NU32_RS232OutBuffer,"Milestone 2\r\n");
    NU32_WriteUART1(NU32_RS232OutBuffer);
    sprintf(LCD_Out_Buffer,"Milestone 2");
    LCDWriteString(LCD_Out_Buffer, 1, 1);

    sprintf(LCD_Out_Buffer,"DC=0");
    LCDWriteString(LCD_Out_Buffer, 2, 1);

    // infinite loop
    while(1)
    {
        if (!NU32USER) // USER is 0 when pushed, 1 when unpushed
            {
                setMotorSpeed(getDutyCycle());
            }
    }
    return 0;
}

//////FUNCTIONS
// initalize the core timer
void initCoreTimer(void)
{
    _CP0_SET_COMPARE(CORE_TICKS);                      // CP0_COMPARE register set to 40 M
    mConfigIntCoreTimer(CT_INT_ON | CT_INT_PRIOR_7);   // enable CT interrupts with IPL7
    WriteCoreTimer(0);                                 // set core timer counter to 0
}

// initialize the analog input
void initAnalogInput(void)
{
    // configure and enable the ADC
    CloseADC10();	// ensure the ADC is off before setting the configuration

    // define setup parameters for OpenADC10
    // Turn module on | output in integer | trigger mode auto | enable  autosample
    #define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // define setup parameters for OpenADC10
    // ADC ref external    | disable offset test    | enable scan mode | perform 1 sample | use one buffer | use MUXA mode
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    // ** If you want to read more than 1 pin, change the 1 in ADC_SAMPLES_PER_INT_1 to the total number of pins you want

    // define setup parameters for OpenADC10
    // 				  use ADC internal clock | set sample time
    #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

    // define setup parameters for OpenADC10
    // set AN0 -> pin B0, ...
    #define PARAM4	ENABLE_AN0_ANA | ENABLE_AN1_ANA
    // ** If you want to read more pins, | (or) them together, like:
    // ** ENABLE_AN3_ANA | ENABLE_AN5_ANA | ENABLE_AN6_ANA
    // ** to read B3, B5 and B6

    // define setup parameters for OpenADC10
    // assign channels you don't want to scan
    #define PARAM5	SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
    // ** This one has all the pins that are not in PARAM4

    // use ground as neg ref for mux A, don't worry about it
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF);
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using parameters define above

    EnableADC10(); // Enable the ADC
}

// Initalize the H-bridge
void initHBridge(void)
{
    LATD  |= 0x0000; // set all of the outputs on D to low
    TRISD &= 0xFE1F; // set pins 5, 6, 7, 8 in D to outputs (0xFF95 = 0b1111111000011111)

    // use Timer2 for a 1kHz PWM signal for the motor on the h-bridge
    OpenTimer2(T2_ON | T2_PS_1_64, 1250); // 80000000Hz / 64ps / 1000Hz = 1250
                                            // 1250 is < 2^16-1, so it is good to use
                                            // this also means that 1250 corresponds to 100% duty cycle

    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // pin D0
    SetDCOC1PWM(MAX_PWM); // duty cycle from 0-1250

    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // pin D1
    SetDCOC2PWM(MAX_PWM); // duty cycle from 0-1250

    // use Timer3 for a 50Hz PWM signal for the RC servo motor
    OpenTimer3(T3_ON | T2_PS_1_64, 25000); // 80000000Hz / 64ps / 50Hz = 25000
                                         // 25000 is < 2^16-1, so it is good to use
                                         // this also means that 25000 corresponds to 100% duty cycle
    OpenOC3(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // pin D2
    int ServoDuty = 850; // 30 degrees -> (3000 counts / 180 degrees) * 30 degrees + 300 min val = 800
    SetDCOC3PWM(ServoDuty); // duty cycle from 0-25000
                  // the rc servo has a minimum pulse of 0.24ms for 0 deg and a max of 2.6ms for 180 deg
                  // at 1/(80000000/64)s per pulse, this corresponds to a min value of 300 and a max of 3300
                  // and each value has a resolution of (180 degree / 3000 values) = 0.06 degrees

    // Turn Motors ON
    EN1 = 1;
    EN2 = 1;

    // Drive Forward
    DIR1 = 0;
    DIR2 = 0;

    // initalize speed
    setMotorSpeed(0);
}

int getDutyCycle()
{
    // read the analog value on B0 (the potentiometer)
    float potValue = ReadADC10(0);

    // Scale the duty cycle so 1023 = 1250 (100% PWM)
    int dutyCycle = potValue * 1.222;

    sprintf(LCD_Out_Buffer,"DC=%4d", dutyCycle);
    LCDWriteString(LCD_Out_Buffer, 2, 1);

    return dutyCycle;
}

int getPTValue()
{
    int ptValue = ReadADC10(1);

    if (ptValue > 100)
        sprintf(LCD_Out_Buffer,"!!!!!");
    else
        sprintf(LCD_Out_Buffer,"     ");
    LCDWriteString(LCD_Out_Buffer, 2, 11);

    return ptValue;

}

void setMotorSpeed(int dutyCycle)
{
    static unsigned int speed = 0;
    if (dutyCycle >= 0)
        speed = dutyCycle;
    
    if (DIR1 == 0)
        SetDCOC1PWM(speed);
    else
        SetDCOC1PWM(MAX_PWM - speed);
    if (DIR2 == 0)
        SetDCOC2PWM(speed);
    else
        SetDCOC2PWM(MAX_PWM - speed);

    // print to LCD
    if (DIR1 == 0)
        sprintf(LCD_Out_Buffer,"-->");
    else
        sprintf(LCD_Out_Buffer,"<--");
    LCDWriteString(LCD_Out_Buffer, 1, 13);
}

void reverseDirection()
{
    // Reverse Direction
    DIR1 = !DIR1;
    DIR2 = !DIR2;

    setMotorSpeed(-1); // alter pwm since direction reversed
}

//ISRs
void __ISR(_CORE_TIMER_VECTOR, IPL7SRS) CoreTimerISR(void)
{  
    time++;

    getPTValue();

    WriteCoreTimer(0);                                 // set core timer counter to 0
    _CP0_SET_COMPARE(CORE_TICKS);                      // must set CP0_COMPARE again after interrupt
    INTClearFlag(_CORE_TIMER_IRQ);                     // clear the interrupt flag
}

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
        if (data == 'e')
        {
            EN1 = !EN1;
            EN2 = !EN2;
        }

        if (data == 's')
        {
            int i = 0;
            int delay = 0;

            // move the servo from 800 to 2000 (30 to 120 degrees)
            // read the phototransistor on pin B1 at each step and save it in a buffer
            for (i=0; i<2050; i++)
            {
                SetDCOC3PWM(i+850);
                for (delay=0; delay<3000; delay++) {} // wait for the rc servo to move
            }

            // move the servo back to 30 degrees
            SetDCOC3PWM(850);
        }

        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
    }

    // We don't care about TX interrupt
    if(INTGetFlag(INT_SOURCE_UART_TX(UART1)))
    {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}
