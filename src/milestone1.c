//////LIBRARIES
#include <plib.h> // for PIC specific functions
#include "NU32.h" // for NU32 board specific functions
#include "LCD.h" // for the 16x2 LCD

//////GLOBAL VARIABLES
// none used here

//////FUNCTION PROTOTYPES
void initAnalogInput(void);
void initPWM(void);

//////MAIN
int main(void)
{
    startup(); // from NU32.c, maximizes performance
    initialize(); // from NU32.c, inits USER, L1 and L2, serial fns

    // init everything for the LCD
    setupLCD();
    LCDClear(0); // clear the LCD in case it still says something

    // turn on analog input and PWM output
    initAnalogInput(); // in this case B3
    initPWM(); // in this case D0

    sprintf(RS232_Out_Buffer,"Milestone 1\r\n");
    WriteString(UART1, RS232_Out_Buffer);
    sprintf(LCD_Out_Buffer,"Milestone 1");
    LCDWriteString(LCD_Out_Buffer, 1, 1);

    int pot_value = 0;

    // infinite loop
    while(1)
    {
        // if the USER switch is pushed, turn on L1, read potentiometer on B3,
        //   change brightness of LED on D0
        // USER is 0 when pushed, 1 when unpushed
        if (!USER)
        {
            // USER is pushed
            //L1 is on when low, is off when high
            L1 = 0;

            // the ReadADC10 fn as reading the index of an array of the B pins
            // you turned on
            pot_value = ReadADC10(0); // retuns a number 0-1023

            // Timer2 rolls over when it counts to 1000, so the possible
            // duty cycles are 0-1000
            // so we need to make sure the pot value is in the right range
            if (pot_value > 1000)
                pot_value = 1000;
            SetDCOC1PWM(pot_value);

            // print the pot value to the LCD
            sprintf(LCD_Out_Buffer,"Pot = %4d", pot_value);
            LCDWriteString(LCD_Out_Buffer, 2, 1);

            // plot the pot value
            sprintf(RS232_Out_Buffer,"%d \n", pot_value/2);
            WriteString(UART1, RS232_Out_Buffer);
        }
        else
            L1 = 1; // L1 is off
    }
    return 0;
}

//////FUNCTIONS
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
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    // ** If you want to read more than 1 pin, change the 1 in ADC_SAMPLES_PER_INT_1 to the total number of pins you want

    // define setup parameters for OpenADC10
    // 				  use ADC internal clock | set sample time
    #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

    // define setup parameters for OpenADC10
    // set AN0 -> pin B0, ...
    #define PARAM4	ENABLE_AN0_ANA
    // ** If you want to read more pins, | (or) them together, like:
    // ** ENABLE_AN3_ANA | ENABLE_AN5_ANA | ENABLE_AN6_ANA
    // ** to read B3, B5 and B6

    // define setup parameters for OpenADC10
    // assign channels you don't want to scan
    #define PARAM5	SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
    // ** This one has all the pins that are not in PARAM4

    // use ground as neg ref for mux A, don't worry about it
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF);
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using parameters define above

    EnableADC10(); // Enable the ADC
}

// initialize the PWM
void initPWM(void)
{
    // Use Timer2 as the base frequency
    OpenTimer2(T2_ON | T2_PS_1_4, 1000); // for 20kHz PWM: 80000000Hz / 4ps / 1000 = 20 kHz
    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // PWM output on Pin D0, 0 duty cycle
    SetDCOC1PWM(0); // unsigned int duty from 0-1000
}

//////ISRs
// interrupt when the NU32 gets a character from the computer
void __ISR(_UART_1_VECTOR, ipl2) IntUart1Handler(void)
{
    // Is this an RX interrupt?
    if(INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {

        char data = UARTGetDataByte(UART1);
        PutCharacter(UART1,data);
        L1 = !L1;
        L2 = !L2;

        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
    }

    // We don't care about TX interrupt
    if(INTGetFlag(INT_SOURCE_UART_TX(UART1)))
    {
        INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
}

