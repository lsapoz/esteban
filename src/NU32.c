#include <plib.h>
#include "NU32.h"

#define SYS_FREQ 80000000
#define DESIRED_BAUDRATE_NU32 115200 // Baudrate

void startup()
//Perform startup routines
{
    SYSTEMConfig(SYS_FREQ, SYS_CFG_ALL);
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    INTEnableSystemMultiVectoredInt();
}

void initialize()
//Perform initial initialization
{
    initializeLEDs();
    initializeSerialPort();
}

void initializeLEDs()
{
    TRISACLR = 0x0030; // A5 and A4 (L2 and L1)
    L1 = 1; // off
    L2 = 0; // on
}

// Initialize the serial port
// Note: the NU32 is hard wired to use UART1 for communication and UART4 for
// bootloading, both are set up with interrupt on rx
void initializeSerialPort() {
  int pbClk;
  // Configure the system performance
  pbClk = SYSTEMConfigPerformance(SYS_FREQ);

  UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART1, pbClk, DESIRED_BAUDRATE_NU32);
  UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // Configure UART1 RX Interrupt
  INTEnable(INT_U1RX, INT_ENABLED);
  INTSetVectorPriority(INT_UART_1_VECTOR, INT_PRIORITY_LEVEL_2);
  INTSetVectorSubPriority(INT_UART_1_VECTOR, INT_SUB_PRIORITY_LEVEL_0);
  
  UARTConfigure(UART4, UART_ENABLE_PINS_TX_RX_ONLY);
  UARTSetFifoMode(UART4, UART_INTERRUPT_ON_TX_DONE | UART_INTERRUPT_ON_RX_NOT_EMPTY);
  UARTSetLineControl(UART4, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
  UARTSetDataRate(UART4, pbClk, DESIRED_BAUDRATE_NU32);
  UARTEnable(UART4, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

  // Configure UART4 RX Interrupt
  INTEnable(INT_U4RX, INT_ENABLED);
  INTSetVectorPriority(INT_UART_4_VECTOR, INT_PRIORITY_LEVEL_3);
  INTSetVectorSubPriority(INT_UART_4_VECTOR, INT_SUB_PRIORITY_LEVEL_0);
}

// Write a string over the serial port
void WriteString(UART_MODULE id, const char *string)
{
  while(*string != '\0')
  {
  while(!UARTTransmitterIsReady(id));
  UARTSendDataByte(id, (char) *string);
  string++;
  while(!UARTTransmissionHasCompleted(id));
  }
}

// Put a character over the serial port, called by WriteString
void PutCharacter(UART_MODULE id, const char character) {
  while(!UARTTransmitterIsReady(id));
  UARTSendDataByte(id, character);
  while(!UARTTransmissionHasCompleted(id));
}

// do a software reset and go into bootloader mode if you get a 'B'
void __ISR(_UART_4_VECTOR, ipl3) IntUart4Handler(void)
{
    // Is this an RX interrupt?
    if(INTGetFlag(INT_SOURCE_UART_RX(UART4)))
    {

        char data = UARTGetDataByte(UART4);

        L1 = !L1;
        L2 = !L2;

        if (data == 'B')
        {
            sprintf(RS232_Out_Buffer,"Switching to bootloader\r\n");
            WriteString(UART1, RS232_Out_Buffer);

            SoftReset();
        }

        // Clear the RX interrupt Flag
        INTClearFlag(INT_SOURCE_UART_RX(UART4));
    }

    // We don't care about TX interrupt
    if(INTGetFlag(INT_SOURCE_UART_TX(UART4)))
    {
        INTClearFlag(INT_SOURCE_UART_TX(UART4));
    }
}


