/*
 * DS2482.h
 *
 *  Created on: Apr 25, 2020
 *      Author: user
 */

#ifndef INC_DS2482_H_
#define INC_DS2482_H_

#endif /* INC_DS2482_H_ */

#define TRUE            1
#define FALSE           0

#define CONFIG_APU      0x01  // Active Pullup
#define CONFIG_SPU      0x04  // Strong Pullup
#define CONFIG_1WS      0x08  // 1-Wire Speed

#define STATUS_1WB      0x01  // 1-Wire Busy
#define STATUS_PPD      0x02  // Presence Pulse Detect
#define STATUS_SD       0x04  // Short Detected
//#define STATUS_LL       0x08  // Logic Level
//#define STATUS_RST      0x10  // Device Reset
#define STATUS_SBR      0x20  // Single Bit Result
#define STATUS_TSB      0x40  // Triplet Second Bit
#define STATUS_DIR      0x80  // Branch Direction Taken

#define CMD_DRST        0xF0
#define CMD_WCFG        0xD2
#define CMD_CHSL        0xC3
#define CMD_SRP         0xE1
#define CMD_1WRS        0xB4
#define CMD_1WWB        0xA5
#define CMD_1WRB        0x96
#define CMD_1WSB        0x87
#define CMD_1WT         0x78

#define POLL_LIMIT      5

#define MODE_STANDARD   0x00
#define MODE_OVERDRIVE  0x01

#define MODE_STRONG     1

//#define SLAVE_OWN_ADDRESS             0x5A
//
//#define USE_TIMEOUT     1
//
//#if (USE_TIMEOUT == 1)
//#define I2C_SEND_TIMEOUT_TXIS_MS      5
//#define I2C_SEND_TIMEOUT_SB_MS        5
//#define I2C_SEND_TIMEOUT_ADDR_MS      5
//#endif /* USE_TIMEOUT */
//
//#if (USE_TIMEOUT == 1)
//uint32_t Timeout=0; /* Variable used for Timeout management */
//#endif /* USE_TIMEOUT */

#define I2C_ADDRESS     0x30  //0x18

extern I2C_HandleTypeDef hi2c3;

// DS2482 state
int32_t c1WS, cSPU, cPPM, cAPU;
int32_t short_detected;

int32_t DS2482_reset(void);
int32_t DS2482_write_config(uint8_t config);
int32_t DS2482_detect(void);
int32_t DS2482_channel_select(int32_t channel);
int32_t OWReset(void);
//void OWWriteBit(uint8_t sendbit);
//uint8_t OWReadBit(void);
//uint8_t OWTouchBit(uint8_t sendbit);
void OWWriteByte(uint8_t sendbyte);
uint8_t OWReadByte(void);
//void OWBlock(uint8_t *tran_buf, int32_t tran_len);
//uint8_t OWTouchByte(uint8_t sendbyte);
//int32_t OWFirst(void);
//int32_t OWNext(void);
//int32_t OWSearch(void);
//uint8_t DS2482_search_triplet(int32_t search_direction);
//int32_t OWSpeed(int32_t new_speed);
//int32_t OWLevel(int32_t new_level);
//int32_t OWReadBitPower(int32_t applyPowerResponse);
int32_t OWWriteBytePower(int32_t sendbyte);

//int32_t OW_Write_Scratchpad(uint8_t ch, uint8_t *array);
//uint8_t* OW_Read_Scratchpad(uint8_t ch);
//int32_t OW_Copy_Scratchpad(uint8_t ch);
int32_t OW_Copy_Scratchpad(uint8_t ch, uint8_t *array);
uint8_t* OW_Read_Memory(uint8_t ch);
//int32_t OW_Memory_Erase(uint8_t ch);

/* End of Code */
