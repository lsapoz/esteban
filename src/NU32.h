#ifndef __NU32_H
#define __NU32_H

#define SYS_FREQ 80000000
#define DESIRED_BAUDRATE_NU32 115200 // Baudrate for RS232

#define NU32LED1 LATAbits.LATA4
#define NU32LED2 LATAbits.LATA5
#define NU32USER PORTCbits.RC13

// Initialization routines
void NU32_Startup();
void NU32_Initialize();
void NU32_InitializeLEDs();

// RS232 Functions
void NU32_InitializeSerialPort();
void NU32_EnableUART1Interrupt();
void NU32_ReadUART1(char*,int);
void NU32_WriteUART1(const char *string);
void WriteString(UART_MODULE id, const char *string);
void PutCharacter(UART_MODULE id, const char character);

// Buffers
char NU32_RS232OutBuffer[32];  // Buffer for sprintf in serial communication

#endif // __NU32_H

