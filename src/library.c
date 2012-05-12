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

// set a 250Hz timer interrupt
void initTimer4Interrupt(void)
{
    OpenTimer4(T4_ON | T4_PS_1_256, TMR4_PR); // 80000000Hz / 256ps / 250Hz = 1250
    mT4SetIntPriority(3); // set Timer4 Interrupt Priority
    mT4ClearIntFlag(); // clear interrupt flag
    mT4IntEnable(1); // enable timer4 interrupts
}

// set a 10Hz timer interrupt
void initTimer5Interrupt(void)
{
    OpenTimer5(T5_ON | T5_PS_1_256, TMR5_PR); // 80000000Hz / 256ps / 10Hz = 31250
    mT5SetIntPriority(3); // set Timer5 Interrupt Priority
    mT5ClearIntFlag(); // clear interrupt flag
    mT5IntEnable(1); // enable timer5 interrupts
}

// initalize the digital inputs with pullup resistors, and turn change notifcation ISR on
void initChangeNotification(void)
{
    mPORTBSetPinsDigitalIn(BIT_0| BIT_1 | BIT_2 | BIT_3); // Make B0, B1, B2, B3 digital inputs
    mCNOpen(CN_ON, CN2_ENABLE | CN3_ENABLE | CN4_ENABLE | CN5_ENABLE, CN2_PULLUP_ENABLE | CN3_PULLUP_ENABLE | CN4_PULLUP_ENABLE | CN5_PULLUP_ENABLE);

    PORTB;  // Clear any mismatches that may already be there by reading the correct ports

    mCNSetIntPriority(3);    // set change notifcation priority
    mCNSetIntSubPriority(2); // set change notification subpriority
    mCNClearIntFlag();
    mCNIntEnable(1);        // enable change notifcation interrupts
}

// initalize the digital output pins 
// Note: motor enable and direction pins initalized in initOutputCompare();
//       encoder slave select pins initalized in initEncoderSPI();
void initDigitalOut(void)
{
    mPORTDSetPinsDigitalOut(BIT_10 | BIT_11 | BIT_12 | BIT_13);  // make D10, D11, D12, D13 digital outs
    LASER_LIGHT = 0;     // pin D10 - intially 0
    LEDS = 0;            // pin D11 - initially 0
    FAN1 = 0;            // pin D12 - initially 0
    FAN2 = 0;            // pin D13 - initially 0
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
    // ADC ref external | disable offset test | enable scan mode | perform 8 samples | use one buffer | use MUXA mode
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_8 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    // ** If you want to read more than 1 pin, change the 1 in ADC_SAMPLES_PER_INT_1 to the total number of pins you want

    // define setup parameters for OpenADC10
    // use ADC internal clock | set sample time
    #define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

    // define setup parameters for OpenADC10
    #define PARAM4	ENABLE_AN6_ANA | ENABLE_AN7_ANA | ENABLE_AN8_ANA | ENABLE_AN9_ANA | ENABLE_AN10_ANA | ENABLE_AN11_ANA | ENABLE_AN12_ANA | ENABLE_AN13_ANA
    // ** If you want to read more pins, | (or) them together, like:
    // ** ENABLE_AN3_ANA | ENABLE_AN5_ANA | ENABLE_AN6_ANA
    // ** to read B3, B5 and B6

    // define setup parameters for OpenADC10
    // assign channels you don't want to scan
    #define PARAM5	SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15
    // ** This one has all the pins that are not in PARAM4

    // use ground as neg ref for mux A, don't worry about it
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF);
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using parameters define above

    EnableADC10(); // Enable the ADC
}

// Initalize the 5 PWM pins + relevant digital outs
void initOutputCompare(void)
{
    LATD  |= 0x0000; // set all of the outputs on D to low
    TRISD &= 0xFE1F; // set pins 5, 6, 7, 8 in D to outputs (0xFF95 = 0b1111111000011111)

    // use Timer2 for a 1kHz PWM signal for the motors on the h-bridge
    OpenTimer2(T2_ON | T2_PS_1_64, 1250); // 80000000Hz / 64ps / 1000Hz = 1250

    // Left Motor - pin D0
    OpenOC1(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    SetDCOC1PWM(0); // duty cycle from 0-1250

    // Right Motor - pin D1
    OpenOC2(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    SetDCOC2PWM(0); // duty cycle from 0-1250

    // use Timer3 for a 50Hz PWM signal for the RC servo motor
    OpenTimer3(T3_ON | T2_PS_1_64, 25000); // 80000000Hz / 64ps / 50Hz = 25000

    // Fan Arm - pin D2
    OpenOC3(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    SetDCOC3PWM(FAN_ARM_VERTICAL); // duty cycle from 0-25000

    // Tower Door - pin D3
    OpenOC4(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    SetDCOC4PWM(TOWER_DOOR_CLOSED); // duty cycle from 0-25000

    // Laser - pin D4
    OpenOC5(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0);
    SetDCOC5PWM(LASER_LEFT); // duty cycle from 0-25000

    // Turn Motors ON
    EN1 = 1;
    EN2 = 1;

    // Drive Forward
    DIR1 = 0;
    DIR2 = 0;

    // initalize speed
    setMotorSpeed(0,0);
}

// initialize SPI communication with the dsPIC at 8MHz + relvant digital outs
// Uses SPI4, pins are: SDI4:F4-RP14, SDO4:F5-RP13, SCK4:F13-RP12
void initEncoderSPI(void) {
    TRISFbits.TRISF12 = 0;      // pin F12 - digital out
    TRISFbits.TRISF3 = 0;       // pin F3 - digital out
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

void rotateFanArm(int servoPos)
{
    SetDCOC3PWM(servoPos);
}

void rotateTowerDoor(int servoPos)
{
    SetDCOC4PWM(servoPos);
}

void scanLaser()
{
    long lastTime;
    int pwm = LASER_LEFT;
    int arrsize = ((LASER_RIGHT-LASER_LEFT)/LASER_STEP);
    int ptON[arrsize], ptOFF[arrsize], ptDIF[arrsize];
    int i = 0;
    
    while (pwm != LASER_RIGHT) {
        ptOFF[i] = ReadADC10(LASER_PT);
        LASER_LIGHT = 1;
        lastTime = time;
        while (time < lastTime + 2){};
        ptON[i] = ReadADC10(LASER_PT);
        LASER_LIGHT = 0;

        pwm += LASER_STEP;
        SetDCOC5PWM(pwm);
        lastTime = time;
        while (time < lastTime + 3){};

        ptDIF[i] = ptON[i] - ptOFF[i];
        sprintf(NU32_RS232OutBuffer,"%d\n", ptDIF[i]/2);
        WriteString(UART1, NU32_RS232OutBuffer);
        i++;
    }
    SetDCOC5PWM(LASER_LEFT);

}

void blowCubeIn()
{
    if (!crateInFront) {
        int crates = numCrates; // record the current number of crates
        long startTime;  // record the current time
        rotateFanArm(FAN_ARM_HORIZONTAL); // rotate fan arm into position
        rotateTowerDoor(TOWER_DOOR_OPEN); // open the tower door
        startTime = time;
        while (time < startTime+200){};   // wait 2/10th of a second
        FAN1 = 1;   // turn on the fan arm's fan
        startTime = time;   // update the time
        while (numCrates==crates) {   // while no new crates have entered the tower
            if (time > startTime+1200)  // time out after 2 seconds
                break;
        }
        FAN1 = 0;   // turn the fan off again
        rotateFanArm(FAN_ARM_VERTICAL); // rotate fan arm back up
        startTime = time;
        while (time < startTime+200){};   // wait 2/10th of a second
        rotateTowerDoor(TOWER_DOOR_CLOSED); // close the tower door
    }
}

void blowCubeUp()
{
    if (!crateInFront) {
        long startTime;  // record the current time
        rotateFanArm(FAN_ARM_HORIZONTAL); // rotate fan arm into position
        rotateTowerDoor(TOWER_DOOR_AJAR); // open the tower door
        time = startTime;
        while (time < startTime+200){};   // wait 2/10th of a second
        FAN1 = 1;   // turn on the fan arm's fan
        startTime = time;   // update the time
        while (time < startTime+700){};
        FAN1 = 0;   // turn the fan off again
        rotateFanArm(FAN_ARM_VERTICAL); // rotate fan arm back up
        startTime = time;
        while (time < startTime+500){};   // wait 5/10th of a second
        rotateTowerDoor(TOWER_DOOR_CLOSED); // close the tower door
    }
}

void blowCubesOut()
{
    long startTime = time;  // record the current time
    FAN2 = 1;   // turn the tower fan on
    while (numCrates != 0) {
        if (time > startTime+3000)  // time out after 3 seconds
                break;
    }
    startTime = time;
    while (time < startTime+300){};   // wait 300 ms
    FAN2 = 0;   // turn the tower fan off
}

void blowCubeInAndOut()
{
    if (!crateInFront) {
        int crates = numCrates; // record the current number of crates
        long startTime;  // record the current time
        rotateFanArm(FAN_ARM_HORIZONTAL); // rotate fan arm into position
        rotateTowerDoor(TOWER_DOOR_OPEN); // open the tower door
        startTime = time;
        while (time < startTime+700){};   // wait 7/10th of a second
        FAN1 = 1;   // turn on the fan arm's fan
        startTime = time;   // update the time
        while (time < startTime + 250){};
        FAN1 = 0;
        startTime = time;
        while (time < startTime + 750){};
        FAN1 = 1;
        startTime = time;
        while (numCrates==crates) {   // while no new crates have entered the tower
            if (time > startTime+2000)  // time out after 2 seconds
                break;
        }
        FAN1 = 0;   // turn the fan off again
        rotateFanArm(FAN_ARM_VERTICAL); // rotate fan arm back up
        while (time < startTime+200){};   // wait 2/10th of a second
        rotateTowerDoor(TOWER_DOOR_CLOSED); // close the tower door
        startTime = time;
        while (time < startTime + 200){};
        FAN2 = 1;   // turn the tower fan on
        startTime = time;
        while (numCrates != 0) {
            if (time > startTime+3000)  // time out after 3 seconds
                    break;
        }
        startTime = time;
        while (time < startTime+300){};   // wait 300 ms
        FAN2 = 0;   // turn the tower fan off
        }
}

// Set the pwm duty cycle that controls motor speed
void setMotorSpeed(int dutyCycle1, int dutyCycle2)
{
    static unsigned int speed1 = 0;
    static unsigned int speed2 = 0;
    if (dutyCycle1 >= 0 && dutyCycle2 >= 0) {
        speed1 = dutyCycle1;
        speed2 = dutyCycle2;
    }
        
    if (DIR1 == 0)
        SetDCOC1PWM(speed1);
    else
        SetDCOC1PWM(MAX_PWM - speed1);
    if (DIR2 == 0)
        SetDCOC2PWM(speed2);
    else
        SetDCOC2PWM(MAX_PWM - speed2);
}

// reverse motor direction while keeping speed constant
void reverseDirection()
{
    // Reverse Direction
    DIR1 = !DIR1;
    DIR2 = !DIR2;

    setMotorSpeed(-1,-1); // alter pwm since direction reversed
}

void driveDistance(float inches)
{
    // reset encoders
    getEncoder1(TRUE);getEncoder2(TRUE);
    getEncoder1(TRUE);getEncoder2(TRUE);

    if (inches >= 0) {
        DIR1 = 0;
        DIR2 = 0;
        drivingState = FORWARD;
        terminalCounts1 = MM_TO_COUNTS(IN_TO_MM(inches))-INERTIA_COUNTS;
    } else {
        DIR1 = 1;
        DIR2 = 1;
        drivingState = BACKWARD;
        terminalCounts1 = MM_TO_COUNTS(IN_TO_MM(-1*inches))-INERTIA_COUNTS;
    }
}

void turnAngle(float degrees)    // 3 = CCW, 4 = CW
{
    // reset encoders
    getEncoder1(TRUE);getEncoder2(TRUE);
    getEncoder1(TRUE);getEncoder2(TRUE);
    
    if (degrees < 0) {
        DIR1 = 1;
        DIR2 = 0;
        drivingState = CCW;
        terminalDegrees = -1*degrees;
    }
    else {
        DIR1 = 0;
        DIR2 = 1;
        drivingState = CW;
        terminalDegrees = degrees;
    }
}

void driveToCenter()
{
    lastColor = currentColor;
    if (lastColor == startingColor) {
        DIR1 = 0;
        DIR2 = 0;
    } else {
        DIR1 = 1;
        DIR2 = 1;
    }
    drivingState = COLOR_SWITCH;
}