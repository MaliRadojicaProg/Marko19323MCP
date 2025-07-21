#include "./sd_routines.h"
#include "./spi.h"
#include <xc.h>
#include "proc/pic18f2520.h"

//******************************************************************
//Function        : to initialize the SD/SDHC card in SPI mode
//Arguments        : none
//return        : unsigned char; will be 0 if no error,
//                           otherwise the response byte will be sent
//******************************************************************

volatile unsigned long startBlock, totalBlocks;
volatile unsigned char SDHC_flag, cardType; //, buffer[512];

unsigned char SD_init(void) {
    unsigned char i, response, SD_version;
    unsigned int retry = 0;
    
    // PORTBbits.RB4 = 1;

    for (i = 0; i < 20; i++)
        SPI1_Write(0xFF); //80 clock pulses spent before sending the first command
    
    //PORTBbits.RB4 = 1;

    SD_CS_ASSERT;

    do {
        response = SD_sendCommand(GO_IDLE_STATE, 0); //send 'reset & go idle' command
        retry++;
        if (retry > 0xFE) return 1; //time out, card not detected
    } while (response != 0x01);

    SD_CS_DEASSERT;
    SPI1_Write(0xFF);
    SPI1_Write(0xFF);
    retry = 0;

    //PORTBbits.RB4 = 1;

    SD_version = 2;
    //default set to SD compliance with ver2.x;
    //this may change after checking the next command
    do {
        response = SD_sendCommand(SEND_IF_COND, 0x000001AA);
        //Check power supply status, mendatory for SDHC card
        retry++;
        if (retry > 0xfe) {
            //TX_NEWLINE; izbacio
            SD_version = 1;
            cardType = 1;
            //return 1; // NOVO
            break;
        } //time out
    } while (response != 0x01);

    retry = 0;


    do {
        response = SD_sendCommand(APP_CMD, 0); //CMD55, must be sent before sending any ACMD command
        response = SD_sendCommand(SD_SEND_OP_COND, 0x40000000); //ACMD41
        retry++;
        if (retry > 0x4fe) {
            //TX_NEWLINE;
            return 2; //time out, card initialization failed
        }
    } while (response != 0x00);

    retry = 0;
    SDHC_flag = 0;


    if (SD_version == 2) {
        do {
            response = SD_sendCommand(READ_OCR, 0);
            retry++;
            if (retry > 0xfe) {
                //TX_NEWLINE; izbacio
                cardType = 0;
                break;
            } //time out
        } while (response != 0x00);

        if (SDHC_flag == 1) cardType = 2;
        else cardType = 3;
    }
    //SD_sendCommand(CRC_ON_OFF, OFF); //disable CRC; deafault - CRC disabled in SPI mode
    //SD_sendCommand(SET_BLOCK_LEN, 512); //set block size to 512; default size is 512

    return 0; //successful return
}

//******************************************************************
//Function        : to send a command to SD card
//Arguments        : unsigned char (8-bit command value)
//                           & unsigned long (32-bit command argument)
//return        : unsigned char; response byte
//******************************************************************

unsigned char SD_sendCommand(unsigned char cmd, unsigned long arg) {
    unsigned char response, retry, status1;
    retry = 0;

    //SD card accepts byte address while SDHC accepts block address in multiples of 512
    //so, if it's SD card we need to convert block address into corresponding byte address by
    //multipying it with 512. which is equivalent to shifting it left 9 times
    //following 'if' loop does that

    if (SDHC_flag == 0)
        if ((cmd == READ_SINGLE_BLOCK) ||
                (cmd == READ_MULTIPLE_BLOCKS) ||
                (cmd == WRITE_SINGLE_BLOCK) ||
                (cmd == WRITE_MULTIPLE_BLOCKS) ||
                (cmd == ERASE_BLOCK_START_ADDR) ||
                (cmd == ERASE_BLOCK_END_ADDR)) {
            arg = (arg << 9);
        }
    SD_CS_ASSERT;

    SPI1_Write(cmd | 0x40); //send command, first two bits always '01'
    SPI1_Write(arg >> 24);
    SPI1_Write(arg >> 16);
    SPI1_Write(arg >> 8);
    SPI1_Write(arg);

    if (cmd == SEND_IF_COND) //it is compulsory to send correct CRC for CMD8 (CRC=0x87) & CMD0 (CRC=0x95)
        SPI1_Write(0x87); //for remaining commands, CRC is ignored in SPI mode
    else
        SPI1_Write(0x95);

    while ((response = SPI1_Read(0xFF)) == 0xFF) //wait response
        if (retry++ > 0xfe) break; //time out error

    if ((response == 0x00) && (cmd == 58)) //checking response of CMD58
    {
        status1 = (SPI1_Read(0xFF) & 0x40); //first byte of the OCR register (bit 31:24)
        if (status1 == 0x40) SDHC_flag = 1; //we need it to verify SDHC card
        else SDHC_flag = 0;

        SPI1_Read(0xFF); //remaining 3 bytes of the OCR register are ignored here
        SPI1_Read(0xFF); //one can use these bytes to check power supply limits of SD
        SPI1_Read(0xFF);
    }

    SPI1_Read(0xFF); //extra 8 CLK
    SD_CS_DEASSERT;

    return response; //return state
}

//*****************************************************************
//Function        : to erase specified no. of blocks of SD card
//Arguments        : none
//return        : unsigned char; will be 0 if no error,
//                           otherwise the response byte will be sent
//*****************************************************************

unsigned char SD_erase(unsigned long startBlock, unsigned long totalBlocks) {
    unsigned char response;

    response = SD_sendCommand(ERASE_BLOCK_START_ADDR, startBlock);
    //send starting block address
    if (response != 0x00) //check for SD status: 0x00 - OK (No flags set)
        return response;

    response = SD_sendCommand(ERASE_BLOCK_END_ADDR, (startBlock + totalBlocks - 1));
    //send end block address
    if (response != 0x00)
        return response;

    response = SD_sendCommand(ERASE_SELECTED_BLOCKS, 0);
    //erase all selected blocks
    if (response != 0x00)
        return response;

    return 0; //normal return
}

//******************************************************************
//Function        : to read a single block from SD card
//Arguments        : none
//return        : unsigned char; will be 0 if no error,
//                           otherwise the response byte will be sent
//******************************************************************

unsigned char SD_readSingleBlock(unsigned long startBlock, unsigned char * Buffer1) {
    unsigned char response;
    unsigned int i, retry = 0;

    response = SD_sendCommand(READ_SINGLE_BLOCK, startBlock);
    //read a Block command

    if (response != 0x00) return response;
    //check for SD status: 0x00 - OK (No flags set)
    SD_CS_ASSERT;

    retry = 0;
    while (SPI1_Read(0xFF) != 0xfe) // wait for start block token 0xfe (0x11111110)
        if (retry++ > 0xFFfe) {
            SD_CS_DEASSERT;
            return 1;
        } //return if time-out

    for (i = 0; i < 512; i++) //read 512 bytes
        Buffer1[i] = SPI1_Read(0xFF);

    SPI1_Read(0xFF); //receive incoming CRC (16-bit), CRC is ignored here
    SPI1_Read(0xFF);

    SPI1_Read(0xFF); //extra 8 clock pulses
    SD_CS_DEASSERT;

    return 0;
}

//******************************************************************
//Function        : to write to a single block of SD card
//Arguments        : none
//return        : unsigned char; will be 0 if no error,
//                           otherwise the response byte will be sent
//******************************************************************

unsigned char SD_writeSingleBlock(unsigned long startBlock, unsigned char * Buffer1) {
    unsigned char response;
    unsigned int i, retry = 0;

    response = SD_sendCommand(WRITE_SINGLE_BLOCK, startBlock);
    //write a Block command
    if (response != 0x00) return response;
    //check for SD status: 0x00 - OK (No flags set)
    SD_CS_ASSERT;

    SPI1_Write(0xfe); //Send start block token 0xfe (0x11111110)
    for (i = 0; i < 512; i++) //send 512 bytes data
        SPI1_Write(Buffer1[i]);

    SPI1_Write(0xFF); //transmit dummy CRC (16-bit), CRC is ignored here
    SPI1_Write(0xFF);

    response = SPI1_Read(0xFF);
    if ((response & 0x1f) != 0x05)
        //response= 0xXXX0AAA1 ; AAA='010' - data accepted
    { //AAA='101'-data rejected due to CRC error
        SD_CS_DEASSERT; //AAA='110'-data rejected due to write error
        return response;
    }

    while (!SPI1_Read(0xFF)) //wait for SD card to complete writing and get idle
        if ((retry++) > 0xFFfe) {
            SD_CS_DEASSERT;
            return 1;
        }

    SD_CS_DEASSERT;
    SPI1_Write(0xFF); //just spend 8 clock cycle delay before reasserting the CS line
    SD_CS_ASSERT; //re-asserting the CS line to verify if card is still busy

    while (!SPI1_Read(0xFF)) //wait for SD card to complete writing and get idle
        if (retry++ > 0xFFfe) {
            SD_CS_DEASSERT;
            return 1;
        }
    SD_CS_DEASSERT;
    return 0;
}

//***************************************************************************
//Function        : to read multiple blocks from SD card & send every block to UART
//Arguments        : none
//return        : unsigned char; will be 0 if no error,
//                           otherwise the response byte will be sent
//****************************************************************************

unsigned char SD_readMultipleBlock(unsigned long startBlock, unsigned char * Buffer1, unsigned char * Buffer2) {
    unsigned char totalBlocks;
    unsigned char response;
    unsigned int i, retry;
    unsigned int index;

    totalBlocks = 4;
    retry = 0;

    response = SD_sendCommand(READ_MULTIPLE_BLOCKS, startBlock);
    //write a Block command
    if (response != 0x00) return response;
    //check for SD status: 0x00 - OK (No flags set)
    SD_CS_ASSERT;

    while (totalBlocks) {
        retry = 0;
        while (SPI1_Read(0xFF) != 0xfe)
            //wait for start block token 0xfe (0x11111110)
            if (retry++ > 0xFFfe) {
                SD_CS_DEASSERT;
                return 1;
            }
        //return if time-out

        if ((totalBlocks == 2) || (totalBlocks == 4)) index = 0;
        else index = 512;

        if (totalBlocks > 2)
            for (i = index; i < index + 512; i++) //read 512 bytes data
                Buffer1[i] = SPI1_Read(0xFF);
        else
            for (i = index; i < index + 512; i++) //read 512 bytes data
                Buffer2[i] = SPI1_Read(0xFF);

        //for(i=0; i<512; i++) //read 512 bytes
        //  buffer[i] = SPI1_Read();

        SPI1_Read(0xFF); //receive incoming CRC (16-bit), CRC is ignored here
        SPI1_Read(0xFF);
        SPI1_Read(0xFF); //extra 8 cycles
        totalBlocks--;
    }

    SD_sendCommand(STOP_TRANSMISSION, 0); //command to stop transmission
    SD_CS_DEASSERT;
    SPI1_Read(0xFF); //extra 8 clock pulses

    return 0;
}

//***************************************************************************
//Function: to receive data from UART and write to multiple blocks of SD card
//Arguments: none
//return: unsigned char; will be 0 if no error,
// otherwise the response byte will be sent
//****************************************************************************

unsigned char SD_writeMultipleBlock(unsigned long startBlock, unsigned char upis_Buffer1,
        unsigned char * Buffer1, unsigned char * Buffer2) {

    unsigned char totalBlocks;
    unsigned char response;
    unsigned int i, retry;
    unsigned long blockCounter;
    unsigned int index;

    retry = 0;
    totalBlocks = 2;
    blockCounter = 0;
    response = SD_sendCommand(WRITE_MULTIPLE_BLOCKS, startBlock); //write a Block command
    if (response != 0x00) return response; //check for SD status: 0x00 - OK (No flags set)

    SD_CS_ASSERT;


    while (blockCounter < totalBlocks) {
        SPI1_Write(0xfc);
        //Send start block token 0xfc (0x11111100)

        if (blockCounter == 0) index = 0;
        else index = 512;

        if (upis_Buffer1 == 0x00) // bilo je 0x01  -- promena
            for (i = index; i < index + 512; i++) //send 512 bytes data
                SPI1_Write(Buffer1[i]);
        else
            for (i = index; i < index + 512; i++) //send 512 bytes data
                SPI1_Write(Buffer2[i]);

        SPI1_Write(0xFF); //transmit dummy CRC (16-bit), CRC is ignored here
        SPI1_Write(0xFF);
        response = SPI1_Read(0xFF);

        if ((response & 0x1f) != 0x05) //response= 0xXXX0AAA1 ; AAA='010' - data accepted
        { //AAA='101'-data rejected due to CRC error
            SD_CS_DEASSERT; //AAA='110'-data rejected due to write error
            return response;
        }

        while (!SPI1_Read(0xFF)) //wait for SD card to complete writing and get idle
            if (retry++ > 0xFFfe) {
                SD_CS_DEASSERT;
                return 1;
            }

        SPI1_Read(0xFF); //extra 8 bits
        blockCounter++;
    }

    SPI1_Write(0xfd); //send 'stop transmission token'
    retry = 0;

    while (!SPI1_Read(0xFF)) //wait for SD card to complete writing and get idle
        if (retry++ > 0xFFfe) {
            SD_CS_DEASSERT;
            return 1;
        }

    SD_CS_DEASSERT;
    // odavde
    SPI1_Write(0xFF); //just spend 8 clock cycle delay before reasserting the CS signal
    SD_CS_ASSERT; //re assertion of the CS signal is required to verify if card is still busy

    while (!SPI1_Read(0xFF)) //wait for SD card to complete writing and get idle
        if (retry++ > 0xFFfe) {
            SD_CS_DEASSERT;
            return 1;
        }
    SD_CS_DEASSERT;
    //dovde
    return 0;
}
