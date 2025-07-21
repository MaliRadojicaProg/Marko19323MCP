/*
 * File:   main.c
 * Author: student
 *
 * Created on November 21, 2022, 12:53 PM
 */

// PIC18F2520 Configuration Bit Settings
// 'C' source line config statements

// CONFIG1H
#pragma config OSC = HS // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))

#pragma config FCMEN = OFF // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF  // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF  // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3    // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF     // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768 // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF   // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF  // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF    // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = OFF // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF    // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF  // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include "proc/pic18f2520.h"
#include <pic18.h>

#include "spi.h"
#include "sd_routines.h"
#define _XTAL_FREQ 25000000

// void main(void) {
//     return;
// }
int addrcalc=0;
int ADC_count = 0;
int ADC_sample[3]; // za rez. AD konv.
unsigned char ByteX[4];
int ADC_sample2[3]; // samo za citanje
unsigned char data_pack[14];

unsigned char Buffer1[512];
unsigned char Buffer2[512];
unsigned char *Buffer = NULL;

unsigned int Buffer_count = 0;  // 0 - 127
unsigned int Buffer_count2 = 0; // broji poslate Buffere
unsigned int Buffer_sel = 0;    // 0  Buffer1 za upis, 1  Buffer2 za upis

unsigned char mode_event = 0;  // izbor moda rada
unsigned char read_event = 0;  // za citanje
unsigned char write_event = 0; // za upis
unsigned char cntTas = 0;
unsigned char FlagUSD = 0;
unsigned char FlagTransmit = 0;
unsigned int Result = 0;

int init_SDMMC()
{
    int ret = 0;
    SPI1_Init(SPI_MASTER_OSC_DIV64, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_ACTIVE_2_IDLE);
    ret = SD_init();
    SPI1_Init(SPI_MASTER_OSC_DIV64, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_ACTIVE_2_IDLE);
    return ret;
}

void init_variables()
{
    ADC_sample[0] = 0;
    ADC_sample[1] = 0;
    ADC_sample[2] = 0;
    ADC_sample2[0] = 0;
    ADC_sample2[1] = 0;
    ADC_sample2[2] = 0;
}

void UART_Init()
{

    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;
    // SPBRG = (F_CPU / baud_rate / 4.0) - 1; // sets baud rate
    SPBRG = 49; // 11; //(int) bps; // store value for 9600 baud rate
    SPBRGH = 0x00;
    TXSTAbits.CSRC = 0; // Don't care for asynchronous mode
    TXSTAbits.TX9 = 0;  // Selects 8-bit data transmission
    TXSTAbits.TXEN = 1; // Enable Data tranmission on RC6 pin
    TXSTAbits.SYNC = 0; // Selects Asynchronous Serial Communication Mode
    TXSTAbits.BRGH = 1; // Default Baud rate speed
    BAUDCONbits.BRG16 = 1;
    RCSTA = 0x90; // Enables UART Communication PORT
}

void init()
{
    int ret = 0;
    TRISA = 0x01;
    PORTA = 0xFF;
    // PORTA.F1  - izlaz  mRST (bluetooth)
    // PORTA.F0  - ulaz AD

    TRISB = 0b00001111;
    PORTB = 0xF0; // LED diode su ugasene
    // PORTB.F7  - LED izlaz
    // PORTB.F6  - LED izlaz
    // PORTB.F5  - LED izlaz
    // PORTB.F4  - LED izlaz
    // PORTB.F3  - Taster ulaz
    // PORTB.F2  - Taster ulaz
    // PORTB.F1  - Taster ulaz
    // PORTB.F0  - Taster ulaz

    // potrebni su pull-down otpornici za PORTB.F3- PORTB.F0

    TRISC = 0b11010100; // data direction register
    PORTC = 0xFF;
    // PORTC.F7  - TXD (bluetooth ili USB) ulaz
    // PORTC.F6  - RXD (bluetooth ili USB)- izlaz
    // PORTC.F5  - SDIN (mikroSD) -izlaz
    // PORTC.F4  - SDOUT (mikroSD) - ulaz
    // PORTC.F3  - SCLK (mikroSD) -izlaz
    // PORTC.F2  - RTS (bluetooth) ulaz
    // PORTC.F1  - CTS (bluetooth) -izlaz
    // PORTC.F0  - CS (mikro SD) - izlaz

    // podesavanje AD konvertora
    ADCON1 = 0x0E;       // AN0 je analogni ulaz
    ADCON2 = 0b10101110; // Right_justified,  12Tad, Tad=64*Tosc
    ADCON0 = 0x01;       // ADON=1

    T0CON = 0b10000100;
    // TMR0ON=1, T08BIT=0 (16-bitni) , T0PS=100  (preskaler 32)
    TMR0H = 0xFC;
    TMR0L = 0xF3;

    // 25MHz/4= 6.25MHz dolazi na preskaler koji je 32
    // iz preskalera 160ns*32=5.12us
    // 3998.72 us= 5.12us*781
    // brojac tajmera treba da ima ciklus brojanja 0xFCF3
    // pocinje da broji od 0xFCF3 i kad dodje do 0x10000 nastaje prekid
    // u prekidnoj rutini ponovo treba da bude upisan 0xFCF3 u TMR0H:TMR0L

    INTCON = 0b11100000; //(GIE, PEIE, TMR0IE)
    // prioritet TMR0 je visok
    INTCON2 = 0b10000100;
    // (nRBPU) disabled,  interrupt falling edge, prioritet TMR0 je visok
    PIE1 = 0x40;
    // samo ADIE=1

    // podesavanje UART-a ; // za PORTB
    UART_Init();

    // TXSTA.TXEN = 1;
    // TXSTA.SYNC = 0;
    // TXSTA.TX9 = 0;

    PORTBbits.RB4 = 0; // dioda se pali
    ret = init_SDMMC();

    if (ret == 0)
        PORTBbits.RB4 = 1; // dioda se gasi
}

void pack(unsigned int Sample)
{
    for (int i = 0; i < 14; i++)
        data_pack[i] = 0x00;
    data_pack[0] = 0xAA;
    data_pack[5] = (unsigned char)(Sample >> 2);
    data_pack[7] = (unsigned char)((Sample & 0x0003) << 2);
}

void transmit()
{
    int i = 0;
    for (i = 0; i < 14; i++)
    {
        TXREG = data_pack[i];
        while (!TXSTAbits.TRMT)
            ;
    }
}

void pack_samples()
{
   // NAPISATI OVDE PROGRAMSKI KOD - ZADATAK 1
  // ADC_sample=>ByteX
  // ADC_sample[0]=Sample0
  // ADC_sample[1]=Sample1
  // ADC_sample[2]=Sample2
  // jedan ADC_sample ima 10 bita a ByteX 8 bita
  // prvih 6 su nule kod ADC_sample
  ByteX[0]= (unsigned char)((ADC_sample[0])>>2); //9:2
  ByteX[1]=  (unsigned char)((ADC_sample[0] & 0x0003) <<6 | (ADC_sample[1] & 0x03f0));
  ByteX[2]=  (unsigned char)((ADC_sample[1] & 0x000f ) <<4 | (ADC_sample[2] & 0x03c0));
  ByteX[3]=  (unsigned char)((ADC_sample[2] & 0x001f) <<2);
}

void unpack_samples()
{
    // NAPISATI OVDE PROGRAMSKI KOD - ZADATAK 1
    ADC_sample2[0]=(unsigned int)((ByteX[0]<<2) | (ByteX[1] >>6);
    ADC_sample2[1]=(unsigned int)((ByteX[1] & 0x3f)<<2  | (ByteX[2] & 0xf0));
    ADC_sample2[2]=(unsigned int)((ByteX[2] & 0x0f) <<4 | (ByteX[3] & 0xfc)); 
}

void writeBuffer(unsigned int Result)
{
    ADC_sample[ADC_count] = Result;

    if (ADC_count == 2)
        ADC_count = 0;
    else
        ADC_count++;

    if (ADC_count == 0)
    {
        pack_samples(); // ADC_sample[2:0] daje ByteX[3:0]

        if (Buffer_sel == 1)
            Buffer = Buffer2;
        else
            Buffer = Buffer1;
        // NAPISATI OVDE PROGRAMSKI KOD - ZADATAK 2
          Buffer[(Buffer_count<<2)+0]=ByteX[0];
          Buffer[(Buffer_count<<2)+1]=ByteX[1];
          Buffer[(Buffer_count<<2)+2]=ByteX[2];
          Buffer[(Buffer_count<<2)+3]=ByteX[3];
        if (Buffer_count < 127)
        {
            Buffer_count++;
        }
        else
        {
            Buffer_count = 0;

            Buffer_sel = 1 - Buffer_sel; // menja se niz

            if (Buffer_sel == 1)
                PORTBbits.RB6 = 1;
            else
                PORTBbits.RB6 = 0;

            Buffer_count2++;

            if (Buffer_count2 >= 10)
            {
                write_event = 0; // kraj za upis
                PORTBbits.RB6 = 1;
            }
            FlagUSD = 1; // buffer je spreman, treba da se posalje
        }
    }
}

void __interrupt(low_priority) ISR()
{

    if ((PIE1bits.ADIE == 1) && (PIR1bits.ADIF == 1))
    {
        PIR1bits.ADIF = 0; //Konverzija nije gotova
       // NAPISATI OVDE PROGRAMSKI KOD - ZADATAK 3
      // prekid AD konverzija
      Result=(ADRESH <<8) | ADRESL);
      if(mode_event==0){
      FlagTransmit=1;
      }
      else if(write_event == 1){
        writeBuffer(Result);
      }
    }

    if (INTCONbits.TMR0IF == 1)
    {
        // 4ms
        INTCONbits.TMR0IF = 0;
        ADCON0bits.GO_DONE = 1; // start za AD konv

        if (cntTas > 0)
            cntTas--;

        if ((PORTBbits.RB0 == 1) && (cntTas == 0) && (write_event == 0) && (read_event == 0))
        {
            mode_event = 1 - mode_event; // promena rada

            if (mode_event == 0)
                PORTBbits.RB4 = 1; // ONLINE
            else
            {
                PORTBbits.RB4 = 0;
                ADC_count = 0;
                Buffer_sel = 0;
                Buffer_count2 = 0;
                Buffer_count = 0;
            }
            cntTas = 100;
        }

        if ((PORTBbits.RB1 == 1) && (cntTas == 0) && (mode_event == 1) && (write_event == 0) && (read_event == 0))
        {
            read_event = 1;    // za  startovanje citanja
            PORTBbits.RB5 = 0; // u toku citanje
            cntTas = 100;
        }

        if ((PORTBbits.RB2 == 1) && (cntTas == 0) && (mode_event == 1) && (write_event == 0) && (read_event == 0))
        {
            write_event = 1;   // za  startovanje upisa
            PORTBbits.RB6 = 0; // u toku upis
            ADC_count = 0;
            Buffer_sel = 0;
            Buffer_count2 = 0;
            Buffer_count = 0;
            cntTas = 100;
        }

        TMR0H = 0xFC;
        TMR0L = 0xF3;
    }
}

int main()
{
    int adr=0;
    int acR=0;
    unsigned long startBlock = 510;
    unsigned long address = 0;

    init();
    init_variables();

    while (1)
    {
        if (FlagUSD == 1)
        {
            FlagUSD = 0;
            if (Buffer_sel == 1)
                SD_writeSingleBlock(startBlock + Buffer_count2, Buffer1);
            else
                SD_writeSingleBlock(startBlock + Buffer_count2, Buffer2);
        }

        if (FlagTransmit == 1) // ONLINE
        {
            FlagTransmit = 0;
            pack(Result);
            transmit();
        }

        if (read_event == 1)
        {
            Buffer_count = 0;
            for (address = startBlock + 1; address < startBlock + 10; address++)
            {
                SD_readSingleBlock(address, Buffer1);
                // NAPISATI OVDE PROGRAMSKI KOD - ZADATAK 4
                for(adr=0;adr<127;adr++){
                    for(int i=0;i<=3;i++){
                       ByteX[i]=Buffer1[(adr<<2)+i];
                    }
                      unpack_samples();
                    for(int i=0;i<=2;i++){
                      pack(ADC_sample2[i]);
                      transmit();
                      __delay_ms(4);
                    }
                }
               
                            }
            read_event = 0;
            PORTBbits.RB5 = 1;
        }
    }
    return 0;
}
