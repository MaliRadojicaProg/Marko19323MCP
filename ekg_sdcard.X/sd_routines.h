#ifndef _SD_ROUTINES_H_
#define _SD_ROUTINES_H_

#define FAT_TESTING_ONLY         

//use following macros if PORTC0 pin is used for Chip Select of SD
#define SD_CS_ASSERT     PORTC &= 0xFE
#define SD_CS_DEASSERT   PORTC |= 0x01

//SD commands, many of these are not used here
#define GO_IDLE_STATE            0
#define SEND_OP_COND             1
#define SEND_IF_COND             8
#define SEND_CSD                 9
#define STOP_TRANSMISSION        12
#define SEND_STATUS              13
#define SET_BLOCK_LEN            16
#define READ_SINGLE_BLOCK        17
#define READ_MULTIPLE_BLOCKS     18
#define WRITE_SINGLE_BLOCK       24
#define WRITE_MULTIPLE_BLOCKS    25
#define ERASE_BLOCK_START_ADDR   32
#define ERASE_BLOCK_END_ADDR     33
#define ERASE_SELECTED_BLOCKS    38
#define SD_SEND_OP_COND          41   //ACMD
#define APP_CMD                  55
#define READ_OCR                 58
#define CRC_ON_OFF               59


#define ON     1
#define OFF    0

unsigned char SD_init(void);
unsigned char SD_sendCommand(unsigned char cmd, unsigned long arg);
unsigned char SD_readSingleBlock(unsigned long startBlock, unsigned char * Buffer1);
unsigned char SD_writeSingleBlock(unsigned long startBlock, unsigned char * Buffer1);
unsigned char SD_readMultipleBlock(unsigned long startBlock, unsigned char * Buffer1, unsigned char * Buffer2);
unsigned char SD_writeMultipleBlock(unsigned long startBlock, unsigned char upis_Buffer1, unsigned char * Buffer1, unsigned char * Buffer2);
unsigned char SD_erase(unsigned long startBlock, unsigned long totalBlocks);

#endif
