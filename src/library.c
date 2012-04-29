#include <plib.h>
#include "data.h"
#include "library.h"
#include "NU32.h"
#include "LCD.h"
#include "math.h"

// initalize the core timer
void initCoreTimer(void)
{
    _CP0_SET_COMPARE(CORE_TICKS);                      // CP0_COMPARE register set to 40 M
    mConfigIntCoreTimer(CT_INT_ON | CT_INT_PRIOR_7);   // enable CT interrupts with IPL7
    WriteCoreTimer(0);                                 // set core timer counter to 0
}

void initTimer4Interrupt(void)
{
    // set a 250Hz timer interrupt
    OpenTimer4(T4_ON | T4_PS_1_256, TMR4_PR); // 80000000Hz / 256ps / 250Hz = 1250
    mT4SetIntPriority(3); // set Timer4 Interrupt Priority
    mT4ClearIntFlag(); // clear interrupt flag
    mT4IntEnable(1); // enable timer4 interrupts
}

void initTimer5Interrupt(void)
{
    // set a 10Hz timer interrupt
    OpenTimer5(T5_ON | T5_PS_1_256, TMR5_PR); // 80000000Hz / 256ps / 10Hz = 31250
    mT5SetIntPriority(3); // set Timer5 Interrupt Priority
    mT5ClearIntFlag(); // clear interrupt flag
    mT5IntEnable(1); // enable timer5 interrupts
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
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    // ** If you want to read more than 1 pin, change the 1 in ADC_SAMPLES_PER_INT_1 to the total number of pins you want

    // define setup parameters for OpenADC10
    // 				  use ADC internal clock | set sample time
    #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

    // define setup parameters for OpenADC10
    // set AN0 -> pin B0, ...
    #define PARAM4	ENABLE_AN0_ANA | ENABLE_AN1_ANA | ENABLE_AN2_ANA | ENABLE_AN3_ANA
    // ** If you want to read more pins, | (or) them together, like:
    // ** ENABLE_AN3_ANA | ENABLE_AN5_ANA | ENABLE_AN6_ANA
    // ** to read B3, B5 and B6

    // define setup parameters for OpenADC10
    // assign channels you don't want to scan
    #define PARAM5	SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
    // ** This one has all the pins that are not in PARAM4

    // use ground as neg ref for mux A, don't worry about it
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF);
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using parameters define above

    EnableADC10(); // Enable the ADC
}

// initalize the digital IO
void initDigitalOut(void)
{
    TRISDbits.TRISD12 = 0;            // pin D12 - digital out
    COLLISION1 = 0;                   // pin D12 - initially 0
}

void initChangeNotification(void)
{
    TRISDbits.TRISD13 = 1;                              // pin D13 - digital in
    mCNOpen(CN_ON, CN19_ENABLE, CN19_PULLUP_ENABLE);    // D13-CN19

    
    PORTD;  // Clear any mismatches that may already be there by reading the correct ports
        
    mCNSetIntPriority(3);    // set change notifcation priority
    mCNSetIntSubPriority(2); // set change notification subpriority
    mCNClearIntFlag();
    mCNIntEnable(1);        // enable change notifcation interrupts
}

// initialize SPI communication with the dsPIC at 8MHz with SS enabled
// Uses SPI4, pins are: SDI4:F4-RP14, SDO4:F5-RP13, SCK4:F13-RP12, SS4:F12-RP15
void initEncoderSPI(void) {
    TRISFbits.TRISF12 = 0;      // pin F12 - digital out
    TRISFbits.TRISF3 = 0;      // pin F3 - digital out
    SLAVE_SELECT1 = 1;
    SLAVE_SELECT2 = 1;

    int data = 0;
    SPI4CON = 0; // stop and reset SPI4
    data = SPI4BUF; // clear the rx buffer
    SPI4BRG = 0x4; // bit rate to 8MHz, SPI4BRG = 80000000/(2*desired)-1
    SPI4STATCLR = 0x40; // clear overflow
    SPI4CON = 0x8620; // SPI ON, 16 bit xfer, SMP=1, MSTR
}

// function for getting the current encoder1 counts
long long getEncoder1(int reset) {
    static int curr=0, last=0, data=0;
    static long long total_count=0;
    
    SLAVE_SELECT1 = 0;      //tie slave select low

    SPI4BUF = 1; // request the encoder position
    while (!SPI4STATbits.SPIRBF);
    data = SPI4BUF; // garbage was transfered over, ignore it
    SPI4BUF = 5; // write garbage, but the corresponding read will have the data
    while (!SPI4STATbits.SPIRBF);

    curr = SPI4BUF;

    SLAVE_SELECT1 = 1;      //tie it back high

    if (curr >= last) { // assume CCW
        if (curr-last > 32768)              // assume there is never more than ~4 revolutions between readings
            total_count -= 65536+last-curr; // negative rollover
        else
            total_count += curr-last;       // no rollover
    } else {
        if (last-curr > 32768)              // assume there is never more than ~4 revolutions between readings
            total_count += 65536+curr-last; // positive rollover
        else
            total_count -= last-curr;
    }
    last = curr;

    if (reset)
        total_count = 0;

    return total_count;
}

// function for getting the current encoder2 counts
long long getEncoder2(int reset) {
    static int curr=0, last=0, data=0;
    static long long total_count=0;

    SLAVE_SELECT2 = 0;      //tie slave select low

    SPI4BUF = 1; // request the encoder position
    while (!SPI4STATbits.SPIRBF);
    data = SPI4BUF; // garbage was transfered over, ignore it
    SPI4BUF = 5; // write garbage, but the corresponding read will have the data
    while (!SPI4STATbits.SPIRBF);

    curr = SPI4BUF;

    SLAVE_SELECT2 = 1;      //tie it back high

    if (curr >= last) { // assume CCW
        if (curr-last > 32768)              // assume there is never more than ~4 revolutions between readings
            total_count -= 65536+last-curr; // negative rollover
        else
            total_count += curr-last;       // no rollover
    } else {
        if (last-curr > 32768)              // assume there is never more than ~4 revolutions between readings
            total_count += 65536+curr-last; // positive rollover
        else
            total_count -= last-curr;
    }
    last = curr;

    if (reset)
        total_count = 0;

    return total_count;
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
    SetDCOC1PWM(0); // duty cycle from 0-1250

    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // pin D1
    SetDCOC2PWM(0); // duty cycle from 0-1250

    // use Timer3 for a 50Hz PWM signal for the RC servo motor
    OpenTimer3(T3_ON | T2_PS_1_64, 25000); // 80000000Hz / 64ps / 50Hz = 25000
                                         // 25000 is < 2^16-1, so it is good to use
                                         // this also means that 25000 corresponds to 100% duty cycle

    OpenOC3(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // pin D2
    SetDCOC3PWM(FAN_ARM_VERTICAL); // duty cycle from 0-25000

    OpenOC4(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // pin D3
    SetDCOC4PWM(TOWER_DOOR_CLOSED); // duty cycle from 0-25000

    // Turn Motors ON
    EN1 = 1;
    EN2 = 1;

    // Drive Forward
    DIR1 = 0;
    DIR2 = 0;

    // initalize speed
    setMotorSpeed(0,0);
}

// Set the pwm duty cycle that controls motor speed
void setMotorSpeed(int dC1, int dC2)
{
    static unsigned int speed1 = 0;
    static unsigned int speed2 = 0;
    if (dC1 >= 0 && dC2 >= 0) {
        speed1 = dC1;
        speed2 = dC2;
    }
        
    if (DIR1 == 0)
        SetDCOC1PWM(dC1);
    else
        SetDCOC1PWM(MAX_PWM - dC1);
    if (DIR2 == 0)
        SetDCOC2PWM(dC2);
    else
        SetDCOC2PWM(MAX_PWM - dC2);
}

// reverse motor direction while keeping speed constant
void reverseDirection()
{
    // Reverse Direction
    DIR1 = !DIR1;
    DIR2 = !DIR2;

    setMotorSpeed(-1,-1); // alter pwm since direction reversed
}

void rotateFanArm(int servoPos)
{
    SetDCOC3PWM(servoPos);
}

void rotateTowerDoor(int servoPos)
{
    SetDCOC4PWM(servoPos);
}

void driveDistance(int dir, float inches)
{
    getEncoder1(TRUE);
    getEncoder2(TRUE);
    getEncoder1(TRUE);
    getEncoder2(TRUE);
    if (dir <= 1) {
        DIR1 = 0;
        DIR2 = 0;
        drive = 1;
    } else {
        DIR1 = 1;
        DIR2 = 1;
        drive = 2;
    }
        
    terminalCounts1 = MM_TO_COUNTS(IN_TO_MM(inches))-INERTIA_COUNTS;
}

void turnAngle(int dir, float degrees)    // 3 = CCW, 4 = CW
{
    getEncoder1(TRUE);
    getEncoder2(TRUE);
    getEncoder1(TRUE);
    getEncoder2(TRUE);
    if (dir <= 3) {
        DIR1 = 1;
        DIR2 = 0;
        drive = 3;
    }
    else {
        DIR1 = 0;
        DIR2 = 1;
        drive = 4;
    }
    terminalDegrees = degrees;
    //setMotorSpeed(MAX_PWM);
//    float angle = 360*(WHEEL_RADIUS/WHEELBASE)*(((float)(position1-enc1)-(position2-enc2))/COUNTS_PER_REVOLUTION);
//    while(angle < degrees) {
//        angle = 360*(WHEEL_RADIUS/WHEELBASE)*(((float)(position1-enc1)-(position2-enc2))/COUNTS_PER_REVOLUTION);
//    }
    //setMotorSpeed(0);

}