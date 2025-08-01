
// PIC18F2525 Configuration Bit Settings// 'C' source line config statements
// CONFIG1H
#pragma config OSC = HS // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
// CONFIG2L
#pragma config PWRT = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3 // Brown Out Reset Voltage bits (Minimum setting)
// CONFIG2H
#pragma config WDT = OFF // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768 // Watchdog Timer Postscale Select bits (1:32768)
// CONFIG3H
#pragma config CCP2MX = 0 // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)
/// CONFIG4L
#pragma config STVREN = OFF
// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF
// Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF
// Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode
//disabled((Legacy mode))
// CONFIG5L
#pragma config CP0 = OFF
#pragma config CP1 = OFF
#pragma config CP2 = OFF
//#pragma config CP3 = OFF
// Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
// Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
// Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
// Code Protection bit (Block 3 (006000-007FFFh) not code-protected)// CONFIG5H
#pragma config CPB = OFF
#pragma config CPD = OFF
// Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
// Data EEPROM Code Protection bit (Data EEPROM not code-protected)
// CONFIG6L
#pragma config WRT0 = OFF
#pragma config WRT1 = OFF
#pragma config WRT2 = OFF
//#pragma config WRT3 = OFF // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
// Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
// Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
// Write Protection bit (Block 3 (006000-007FFFh) not write-protected)
// CONFIG6H
#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF // Configuration Register Write Protection bit not write-protected)
// Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
// Data EEPROM Write Protection bit (Data EEPROM not write-protected)
// CONFIG7L
#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF
#pragma config EBTR2 = OFF
//#pragma config EBTR3 = OFF // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads)
// Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads)
// Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads)
// Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads)
// CONFIG7H
#pragma config EBTRB = OFF // Boot Block Table Read Protection bit
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include "proc/pic18f2525.h"
#include <pic18.h>

#define _XTAL_FREQ 25000000


int result_ECG = 0x00;
int result_RESP = 0x00;
unsigned char Flag_4ms = 0x00; //4ms * 250 = 1s
unsigned char cntTimer = 0x00;
unsigned char ByteX[14];


void init_vars(){
    for(int i  = 0; i < 14; i++)
        ByteX[i] = 0;
    ByteX[0] = 0xAA;
}

void UART_Init() {
    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;

    // sets baud rate
    SPBRG = 49; //u knjizi pise 49, u vezbi 6 39
    //SPBRGH = 0;
    TXSTAbits.CSRC = 0;
    //Don't care for asynchronous mode
    TXSTAbits.TX9 = 0;
    //Selects 8-bit data transmission
    TXSTAbits.TXEN = 1;
    //Enable Data tranmission on RC6 pin
    TXSTAbits.SYNC = 0;
    // Selects Asynchronous Serial Communication
    TXSTAbits.BRGH = 1;
    // Default Baud rate speed
    BAUDCONbits.BRG16 = 1;
    // selects only 8 bit register for baud rate
    RCSTA = 0x90;
    // Enables UART Communication PORT
}

void init() {

    TRISA = 0x03;
    PORTA = 0xFF;   //1111 1111
    
    TRISB = 0b00110010;
    PORTB = 0xF0;   //1111 0000 
    
    TRISC = 0b11010000;
    PORTC = 0xFF;   //1111 1111
    
    ADCON1 = 0x0D;
    ADCON2 = 0b10101110; // Right justified,16*Tad,Tad=Tosc*32
    ADCON0 = 0x01; // samo ADON=1

    T0CON = 0b10000100;
    TMR0H = 0xFC;
    TMR0L = 0xF3;
    
    INTCON = 0b10100000;
    INTCON2 = 0b10000100;
    
    PIE1 = 0x00;
    UART_Init();
}

void init_pwm() {

    PR2 = 61;
    
    CCPR2L = 0x1F;
    CCP2CON = 0b00011100;
    T2CON = 0x04;
   
    TRISBbits.RB3 = 0;
   // PORTBbits.RB3 = 0;
}

int konvertovanje (unsigned char kanal){

    // NAPISATI PROGRAMSKI KOD - ZADATAK 1    
    
    ADCON0=0x01 | (kanal <<2);
    __delay_us(2.4);
    ADCON0bits.GO_DONE=1;
    while(ADCON0bits.GO_DONE);
    return (ADRESH<<8|ADRESL);
   
}

void transmit_data(unsigned char data_8b) {
    TXREG = data_8b;
    while (!TXSTAbits.TRMT);
}

void pakIsl()
{
    
    int D1 = 0;
    int Resp = 0;

    D1 = result_ECG;
    Resp = result_RESP;
    ByteX[0]=0xAA;
    ByteX[5]=D1<<6;
    ByteX[7]=D1<<14;
    ByteX[9]=Resp<<6;
    ByteX[10]=(Resp & 0x03)<<4;

    for (int i = 0; i <= 13; i++)
        transmit_data(ByteX[i]);
}

void __interrupt(low_priority) ISR() {
    if((INTCONbits.TMR0IE == 1) && (INTCONbits.TMR0IF == 1)){
       INTCONbits.TMR0IF = 0;
       Flag_4ms = 1;
       TMR0H = 0xFC;
       TMR0L = 0xF3;
    } 
}


void main() {
    init_vars();
    init();
    init_pwm();
    
    while(1){
        if((cntTimer > 0) && (cntTimer < 10)) PORTBbits.RB0 = 1;
        else PORTBbits.RB0 = 0;
        
        if(Flag_4ms == 1){
            Flag_4ms = 0;
            if(PORTBbits.RB4 == 1) cntTimer = 100;
            else if (cntTimer > 0) cntTimer--;
            result_ECG = konvertovanje(0);
            result_RESP = konvertovanje(1);
            pakIsl();
        }
    }
}
