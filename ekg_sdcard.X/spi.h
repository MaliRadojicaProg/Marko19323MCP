#ifndef _SPI_ROUTINES_H_
#define _SPI_ROUTINES_H_


typedef enum
{
    SPI_MASTER_OSC_DIV4 = 0b00100000,
    SPI_MASTER_OSC_DIV16 = 0b00100001,
    SPI_MASTER_OSC_DIV64 = 0b00100010,
    SPI_MASTER_TMR2 = 0b00100011,
    SPI_SLAVE_SS_EN = 0b00100100,
    SPI_SLAVE_SS_DIS = 0b00100101
} Spi_Type;

typedef enum
{
    SPI_DATA_SAMPLE_MIDDLE = 0b00000000,
    SPI_DATA_SAMPLE_END = 0b10000000
} Spi_Data_Sample;

typedef enum
{
    SPI_CLOCK_IDLE_HIGH = 0b00010000,
    SPI_CLOCK_IDLE_LOW = 0b00000000
} Spi_Clock_Idle;

typedef enum
{
    SPI_IDLE_2_ACTIVE = 0b00000000,
    SPI_ACTIVE_2_IDLE = 0b01000000
} Spi_Transmit_Edge;

void SPI1_Init(Spi_Type sType, Spi_Data_Sample sDataSample, Spi_Clock_Idle sClockIdle, Spi_Transmit_Edge sTransmitEdge);
static void SPI1_ReceiveWait();
unsigned char SPI1_DataReady(); // Check whether the data is ready to read
unsigned char SPI1_Read(unsigned char d);          // Read the received data
void SPI1_Write(unsigned char dat); //Write data to SPI bus

#endif
