#include "spi.h"
#include <xc.h>
#include "proc/pic18f2520.h"

void SPI1_Init(Spi_Type sType, Spi_Data_Sample sDataSample, Spi_Clock_Idle sClockIdle, Spi_Transmit_Edge sTransmitEdge)
{
  //TRISCbits.RC5 = 0;
  //TRISCbits.RC3 = 0;
  //TRISCbits.RC4 = 1;

  
  //if (sType & 0b00000100) // If Slave Mode
  //{
  //  SSPSTAT = sTransmitEdge;
  //  TRISCbits.RC3 = 1;
  //}
  //else // If Master Mode
  //{
  //}
  
  SSPSTAT = sDataSample | sTransmitEdge;
  //TRISCbits.RC3 = 0;
  
  SSPCON1 = sType | sClockIdle;
}

static void SPI1_ReceiveWait()
{
  while (!SSPSTATbits.BF)
    ; // Wait for Data Receipt complete
}

unsigned char SPI1_DataReady() // Check whether the data is ready to read
{
  if (SSPSTATbits.BF)
    return 1;
  else
    return 0;
}

unsigned char SPI1_Read(unsigned char d) // Read the received data
{
  unsigned char ch=0x00;
  SSPBUF = d;
  SPI1_ReceiveWait(); // Wait until all bits receive
  return (SSPBUF);  // Read the received data from the buffer
}

void SPI1_Write(unsigned char dat) //Write data to SPI bus
{
  unsigned char ch=0x00;
  SSPBUF = dat;
  SPI1_ReceiveWait(); // Wait until all bits receive
  ch = SSPBUF;
}
