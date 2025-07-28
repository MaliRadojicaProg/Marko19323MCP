
// PIC18F2520 Configuration Bit Settings// 'C' source line config statements
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
#pragma config CCP2MX = PORTC // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
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
#pragma config CP3 = OFF
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
#pragma config WRT3 = OFF // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
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
#pragma config EBTR3 = OFF // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads)
// Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads)
// Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads)
// Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads)
// CONFIG7H
#pragma config EBTRB = OFF // Boot Block Table Read Protection bit
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include "proc/pic18f2520.h"
#include <pic18.h>

#define _XTAL_FREQ 20000000


#define SAMPLE_RED PORTBbits.RB0 // AKTIVAN 1
#define SAMPLE_INFRARED PORTBbits.RB2 // AKTIVAN 1
#define SAMPLE_LIGHT PORTAbits.RA4 // AKTIVAN 1
#define TURN_ON_RED PORTBbits.RB1 // AKTIVAN 0
#define TURN_ON_INFRARED PORTBbits.RB3 // AKTIVAN 0


int cnt = 0;
unsigned char ByteX[14];
unsigned char FLAG = 0;
unsigned int res = 0;
int Led_Count=0;

void UART_Init() {
    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;

    // sets baud rate
    SPBRG = 39;
    SPBRGH = 0;
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

    TRISA = 0x0F;
    TRISB = 0x30;
    TRISC = 0xD4;

    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x40;

    ADCON1 = 0x0B;
    ADCON2 = 0b10110010; // Right justified,16*Tad,Tad=Tosc*32
    ADCON0 = 0x01; // samo ADON=1

    T0CON = 0b10001000;
    TMR0H = 0xFE;
    TMR0L = 0x18;
    
    T1CON = 0b10001001;
    TMR1H=0xFF;
    TMR1L=0xF3;
    UART_Init();
    TURN_ON_RED = 1;
    TURN_ON_INFRARED = 1;
    SAMPLE_RED = 0;
    SAMPLE_INFRARED = 0;
    SAMPLE_LIGHT = 0; // podesavanje prekida
    INTCON = 0xA0; // samo GIE=1, TMR0IE=1
    PIE1=0x01;
}



int read_adc(unsigned char BR_KANALA) {
   // NAPISATI PROGRAMSKI KOD - ZADATAK 1
  int result=0;
  switch (BR_KANALA){
    case 0:
      __delay_us(2.4);
      ADCON0=0b00000011;
    break;
    case 1:
      ADCON0=0b00000111;
      __delay_us(2.4);
    break:
    case 2:
      ADCON0=0b00001011;
      __delay_us(2.4);
    break;
    case 3:
      ADCON0=0b00001111;
     __delay_us(2.4); //temperatura je iznad 25C pa TCOFF=1.2us 
    break;
  }
  if(ADCON0bits.GO_DONE==0){
//Konverzija je zavrsena
    result=(ADRESH << 8) | (ADRESL);
  }
   return result;
}

void transmit_data(unsigned char data_8b) {
    TXREG = data_8b;
    while (!TXSTAbits.TRMT);
}


//unsigned char bajt0 = 0xAA;
//unsigned char bajt1 = 0x00; // reserved
//unsigned char bajt2 = 0x00; // reserved
//unsigned char bajt3 = 0x00; // reserved
//unsigned char bajt4 = 0x00; // reserved
//unsigned char bajt5 = 0x00; // D1(11:4)
//unsigned char bajt6 = 0x00; // D2 (11:4)
//unsigned char bajt7 = 0x00; // D1(3:0)&D2(3:0)
//unsigned char bajt8 = 0x00; // V(11:4)
//unsigned char bajt9 = 0x00; // Resp(11:4)
//unsigned char bajt10 = 0x00; // V(3:0) & Resp(3:0)
//unsigned char bajt11 = 0x00; // Red(11:4) ovo na dole
//unsigned char bajt12 = 0x00; // InfraRed(11:4)
//unsigned char bajt13 = 0x00; // Red(3:0) & Infrared(3:0)

void akvizicija() {

    int Red = 0;
    int Infrared = 0;
    int NRed = 0;
    int NInfrared = 0;
    int i = 0;

    Red = read_adc(0);
    Infrared = read_adc(1);
    NRed = read_adc(2);
    NInfrared = read_adc(3);


    for (i = 0; i < 14; i++) {
        ByteX[i] = 0x00;
    }
	
    // NAPISATI PROGRAMSKI KOD - ZADATAK 2
    ByteX[0]=0xAA;
    ByteX[11]=(Red & 0x07f0) << 5;
    ByteX[12]=(Infrared & 0x07f0) << 5;
    ByteX[13]=((Red & 0x000f)<<4)|(Infrared & 0x000f);
    if (NRed > 922) ByteX[4] += 12; // prst gurnut suvise
    else if (NRed > 204) ByteX[4] += 4; // ok
    else ByteX[4] += 0; // prst izvucen

    if (NInfrared > 922) ByteX[4] += 3; // prst gurnut suvise
    else if (NInfrared > 204) ByteX[4] += 1; // ok
    else ByteX[4] += 0; // prst izvucen
    
    for (i=0; i<14; i++) 
        transmit_data(ByteX[i]);


}

unsigned char Flag1 = 0;
void __interrupt(low_priority) ISR() {
    if ((INTCONbits.TMR0IF)&&(INTCONbits.TMR0IE)) {// Tajmer0interrupt
        INTCONbits.TMR0IF = 0;
        Flag1=1;
        TMR0H = 0xFE;
        TMR0L = 0x18;
		  if((PIR1bits.TMR1IF) && (PIE1bits.TMR1IE)){
        PIR1bits.TMR1IF=0;
        TMR1H=0xFF;
        TMR1L=0x37; //8Mhz/4=>2MHz=>0.5us*200=100us 65535-200=6535=0xff37
      //100us interrupt
        Led_Count++;
        switch (Led_Count){
        case 0:
        TURN_ON_RED=0;
        break;
        case 2:
        SAMPLE_RED=1;
        break;
        case 4:
        SAMPLE_RED=0;
        break;
        case 6:
        TURN_ON_RED=1;
        break;
        case 12:
        SAMPLE_LIGHT=1;
        break;
        case 14:
        SAMPLE_LIGHT=0;
        break;
        case 20:
        TURN_ON_INFRARED=0;
        break;
        case 22:
        SAMPLE_INFRARED=1;
        break;
        case 24:
        SAMPLE_INFRARED=0;
        break;
        case 26:
        TURN_ON_INFRARED=1;
        Led_Count=0;
        break;
      }
    }
		// NAPISATI PROGRAMSKI KOD - ZADATAK 3
       
    }
}

void main(void) {

      init();
      if(Flag1==1){
    akvizicija();
    Flag1=0;
  }
  return 0;
       // NAPISATI PROGRAMSKI KOD - ZADATAK 4
}
