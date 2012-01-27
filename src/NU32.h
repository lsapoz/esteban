#ifndef __NU32_H
#define __NU32_H

#define L1     LATAbits.LATA4
#define L2     LATAbits.LATA5
#define USER    PORTCbits.RC13

void startup();
void initialize();
void initializeLEDs();

// Serial Functions
void initializeSerialPort();
void WriteString(UART_MODULE id, const char *string);
void PutCharacter(UART_MODULE id, const char character);

char RS232_Out_Buffer[32];  // Buffer for sprintf in serial communication

#endif // __NU32_H

