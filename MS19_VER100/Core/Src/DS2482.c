/*
 * DS2482.c
 *
 *  Created on: Apr 24, 2020
 *      Author: dohga3
 */
#include "main.h"

//암호화 키 공개키 고정값
uint8_t key[]={0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};
//초기화 벡터값 램덤형태로 계속 변화를 줘야함.
uint8_t iv[]={0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
//aes 구조체
struct AES_ctx ctx;

//--------------------------------------------------------------------------
// Perform a device reset on the DS2482
//
// Returns: TRUE if device was reset
//          FALSE device not detected or failure to perform reset
//
int32_t DS2482_reset(void)
{
//  uint8_t status;
//
// Device Reset
//   S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
//  [] indicates from slave
//  SS status byte to read to verify state

//  I2C_start();
//  I2C_write(I2C_address| I2C_WRITE, EXPECT_ACK);
//  I2C_write(CMD_DRST, EXPECT_ACK);
//  I2C_rep_start();
//  I2C_write(I2C_address| I2C_READ, EXPECT_ACK);
//  status=I2C_read(NACK);
//  I2C_stop();

  uint8_t aTxBuffer[1];
  uint8_t aRxBuffer[1];

  aTxBuffer[0]=CMD_DRST;

  /*##- Start the transmission process #####################################*/
  /* While the I2C in reception process, user can transmit data through
   "aTxBuffer" buffer */
  /* Timeout is set to 10S */
  while(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)I2C_ADDRESS, aTxBuffer, 1, 1000)!= HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
     When Acknowledge failure occurs (Slave don't acknowledge its address)
     Master restarts communication */
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: DS2482_reset\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Transmit\r\n");
      DBG_PRINT("error: HAL_I2C_ERROR_AF\r\n");
      Error_Handler();
    }
  }

  /*##- Put I2C peripheral in reception process ############################*/
  /* Timeout is set to 10S */
  while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
     When Acknowledge failure occurs (Slave don't acknowledge it's address)
     Master restarts communication */
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: DS2482_reset\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Receive\r\n");
      Error_Handler();
    }
  }

//  // check for failure due to incorrect read back of status
//  return ((status& 0xF7)== 0x10);
  return ((aRxBuffer[0]& 0xF7)== 0x10);
}

//--------------------------------------------------------------------------
// Write the configuration register in the DS2482. The configuration
// options are provided in the lower nibble of the provided config byte.
// The uppper nibble in bitwise inverted when written to the DS2482.
//
// Returns:  TRUE: config written and response correct
//           FALSE: response incorrect
//
int32_t DS2482_write_config(uint8_t config)
{
//  uint8_t read_config;
//
// Write configuration (Case A)
//   S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
//  [] indicates from slave
//  CF configuration byte to write

//  I2C_start();
//  I2C_write(I2C_address| I2C_WRITE, EXPECT_ACK);
//  I2C_write(CMD_WCFG, EXPECT_ACK);
//  I2C_write(config| (~config<< 4), EXPECT_ACK);
//  I2C_rep_start();
//  I2C_write(I2C_address| I2C_READ, EXPECT_ACK);
//  read_config=I2C_read(NACK);
//  I2C_stop();

  uint8_t aTxBuffer[2];
  uint8_t aRxBuffer[1];

  aTxBuffer[0]=CMD_WCFG;
  aTxBuffer[1]=config| (~config<< 4);

  while(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)I2C_ADDRESS, aTxBuffer, 2, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: DS2482_write_config\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Transmit\r\n");
      Error_Handler();
    }
  }

  while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: DS2482_write_config\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Receive\r\n");
      Error_Handler();
    }
  }

  // check for failure due to incorrect read back
//  if(config!= read_config)
  if(config!= aRxBuffer[0])
  {
    // handle error
    // ...
    DS2482_reset();

    DBG_PRINT("error: DS2482_write_config\r\n");
    DBG_PRINT("error: incorrect read back\r\n");

    return FALSE;
  }

  return TRUE;
}

//--------------------------------------------------------------------------
// DS2428 Detect routine that sets the I2C address and then performs a
// device reset followed by writing the configuration byte to default values:
//   1-Wire speed (c1WS) = standard (0)
//   Strong pullup (cSPU) = off (0)
//   Presence pulse masking (cPPM) = off (0)
//   Active pullup (cAPU) = on (CONFIG_APU = 0x01)
//
// Returns: TRUE if device was detected and written
//          FALSE device not detected or failure to write configuration byte
//
//int32_t DS2482_detect(uint8_t addr)
int32_t DS2482_detect(void)
{
  // set global address
//  I2C_address=addr;

// reset the DS2482 ON selected address
  if(!DS2482_reset())
  {
    return FALSE;
  }

  // default configuration
  c1WS=FALSE;
  cSPU=FALSE;
  //cSPU=CONFIG_SPU;
  cPPM=FALSE;
  cAPU=CONFIG_APU;

  // write the default configuration setup
  if(!DS2482_write_config(c1WS| cSPU| cPPM| cAPU))
  {
    return FALSE;
  }

  return TRUE;
}

//--------------------------------------------------------------------------
// Select the 1-Wire channel on a DS2482-800.
//
// Returns: TRUE if channel selected
//          FALSE device not detected or failure to perform select
//
int32_t DS2482_channel_select(int32_t channel)
{
//  uint8_t ch, ch_read, check;
  uint8_t ch, ch_read;

  // Channel Select (Case A)
  //   S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
  //  [] indicates from slave
  //  CC channel value
  //  RR channel read back

//  I2C_start();
//  I2C_write(I2C_address| I2C_WRITE, EXPECT_ACK);
//  I2C_write(CMD_CHSL, EXPECT_ACK);

  switch(channel)
  {
    default:
    case 0:
      ch=0xF0;
      ch_read=0xB8;
      break;
    case 1:
      ch=0xE1;
      ch_read=0xB1;
      break;
    case 2:
      ch=0xD2;
      ch_read=0xAA;
      break;
    case 3:
      ch=0xC3;
      ch_read=0xA3;
      break;
    case 4:
      ch=0xB4;
      ch_read=0x9C;
      break;
    case 5:
      ch=0xA5;
      ch_read=0x95;
      break;
    case 6:
      ch=0x96;
      ch_read=0x8E;
      break;
    case 7:
      ch=0x87;
      ch_read=0x87;
      break;
  };

//  I2C_write(ch, EXPECT_ACK);
//  I2C_rep_start();
//  I2C_write(I2C_address| I2C_READ, EXPECT_ACK);
//  check=I2C_read(NACK);
//  I2C_stop();

  uint8_t aTxBuffer[2];
  uint8_t aRxBuffer[1];

  aTxBuffer[0]=CMD_CHSL;
  aTxBuffer[1]=ch;

  while(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)I2C_ADDRESS, aTxBuffer, 2, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: DS2482_channel_select\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Transmit\r\n");
      Error_Handler();
    }
  }

  while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: DS2482_channel_select\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Receive\r\n");
      Error_Handler();
    }
  }

  // check for failure due to incorrect read back of channel
//  return (check== ch_read);
  return (aRxBuffer[0]== ch_read);
}

//--------------------------------------------------------------------------
// Reset all of the devices on the 1-Wire Net and return the result.
//
// Returns: TRUE(1):  presence pulse(s) detected, device(s) reset
//          FALSE(0): no presence pulses detected
//
int32_t OWReset(void)
{
  uint8_t status;
  int32_t poll_count=0;

  // 1-Wire reset (Case B)
  //   S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
  //                                   \--------/
  //                       Repeat until 1WB bit has changed to 0
  //  [] indicates from slave

//  I2C_start();
//  I2C_write(I2C_address| I2C_WRITE, EXPECT_ACK);
//  I2C_write(CMD_1WRS, EXPECT_ACK);
//  I2C_rep_start();
//  I2C_write(I2C_address| I2C_READ, EXPECT_ACK);
//
//  // loop checking 1WB bit for completion of 1-Wire operation
//  // abort if poll limit reached
//  status=I2C_read(ACK);
//  do
//  {
//    status=I2C_read(status& STATUS_1WB);
//  } while((status& STATUS_1WB)&& (poll_count++< POLL_LIMIT));
//
//  I2C_stop();

  uint8_t aTxBuffer[1];
  uint8_t aRxBuffer[1];

  aTxBuffer[0]=CMD_1WRS;

  while(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)I2C_ADDRESS, aTxBuffer, 1, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: OWReset\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Transmit\r\n");
      Error_Handler();
    }
  }

  if(c1WS== FALSE) delay_ms(1);

  while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: OWReset\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Receive1\r\n");
      Error_Handler();
    }
  }

  status=aRxBuffer[0];
  do
  {
    while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
    {
      if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
      {
        DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
        break;
      }

      if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
      {
        DBG_PRINT("error: OWReset\r\n");
        DBG_PRINT("error: HAL_I2C_Master_Receive2\r\n");
        Error_Handler();
      }
    }
    status=aRxBuffer[0]& STATUS_1WB;
  } while((status& STATUS_1WB)&& (poll_count++< POLL_LIMIT));

  // check for failure due to poll limit reached
  if(poll_count>= POLL_LIMIT)
  {
    // handle error
    // ...
    //DS2482_reset();

    DBG_PRINT("error: OWReset\r\n");
    DBG_PRINT("error: poll limit reached\r\n");

    return FALSE;
  }

  // check for short condition
  if(status& STATUS_SD) short_detected= TRUE;
  else short_detected= FALSE;

  // check for presence detect
  if(status& STATUS_PPD)
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and verify that the
// 8 bits read from the 1-Wire Net are the same (write operation).
// The parameter 'sendbyte' least significant 8 bits are used.
//
// 'sendbyte' - 8 bits to send (least significant byte)
//
// Returns:  TRUE: bytes written and echo was the same
//           FALSE: echo was not the same
//
void OWWriteByte(uint8_t sendbyte)
{
  uint8_t status;
  int32_t poll_count=0;

  // 1-Wire Write Byte (Case B)
  //   S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
  //                                          \--------/
  //                             Repeat until 1WB bit has changed to 0
  //  [] indicates from slave
  //  DD data to write

//  I2C_start();
//  I2C_write(I2C_address| I2C_WRITE, EXPECT_ACK);
//  I2C_write(CMD_1WWB, EXPECT_ACK);
//  I2C_write(sendbyte, EXPECT_ACK);
//  I2C_rep_start();
//  I2C_write(I2C_address| I2C_READ, EXPECT_ACK);
//
//  // loop checking 1WB bit for completion of 1-Wire operation
//  // abort if poll limit reached
//  status=I2C_read(ACK);
//  do
//  {
//    status=I2C_read(status& STATUS_1WB);
//  } while((status& STATUS_1WB)&& (poll_count++< POLL_LIMIT));
//
//  I2C_stop();

  uint8_t aTxBuffer[2];
  uint8_t aRxBuffer[1];

  aTxBuffer[0]=CMD_1WWB;
  aTxBuffer[1]=sendbyte;

  while(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)I2C_ADDRESS, aTxBuffer, 2, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: OWWriteByte\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Transmit\r\n");
      Error_Handler();
    }
  }

  if(c1WS== FALSE) delay_ms(1);

  while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: OWWriteByte\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Receive1\r\n");
      Error_Handler();
    }
  }

  status=aRxBuffer[0];
  do
  {
    while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
    {
      if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
      {
        DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
        break;
      }

      if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
      {
        DBG_PRINT("error: OWWriteByte\r\n");
        DBG_PRINT("error: HAL_I2C_Master_Receive2\r\n");
        Error_Handler();
      }
    }
    status=aRxBuffer[0]& STATUS_1WB;
  } while((status& STATUS_1WB)&& (poll_count++< POLL_LIMIT));

  // check for failure due to poll limit reached
  if(poll_count>= POLL_LIMIT)
  {
    // handle error
    // ...
    //DS2482_reset();

    DBG_PRINT("error: OWWriteByte\r\n");
    DBG_PRINT("error: poll limit reached\r\n");
  }
}

//--------------------------------------------------------------------------
// Send 8 bits of read communication to the 1-Wire Net and return the
// result 8 bits read from the 1-Wire Net.
//
// Returns:  8 bits read from 1-Wire Net
//
uint8_t OWReadByte(void)
{
  uint8_t data, status;
  int32_t poll_count=0;

  /*// 1-Wire Read Bytes (Case C)
   //   S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\
   //                                   \--------/
   //                     Repeat until 1WB bit has changed to 0
   //   Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
   //
   //  [] indicates from slave
   //  DD data read*/

//  I2C_start();
//  I2C_write(I2C_address| I2C_WRITE, EXPECT_ACK);
//  I2C_write(CMD_1WRB, EXPECT_ACK);
//  I2C_rep_start();
//  I2C_write(I2C_address| I2C_READ, EXPECT_ACK);
//
//  // loop checking 1WB bit for completion of 1-Wire operation
//  // abort if poll limit reached
//  status=I2C_read(ACK);
//  do
//  {
//    status=I2C_read(status& STATUS_1WB);
//  } while((status& STATUS_1WB)&& (poll_count++< POLL_LIMIT));
  uint8_t aTxBuffer[2];
  uint8_t aRxBuffer[1];

  aTxBuffer[0]=CMD_1WRB;

  while(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)I2C_ADDRESS, aTxBuffer, 1, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: OWReadByte\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Transmit1\r\n");
      Error_Handler();
    }
  }

  if(c1WS== FALSE) delay_ms(1);

  while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: OWReadByte\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Receive1\r\n");
      Error_Handler();
    }
  }

  status=aRxBuffer[0];
  do
  {
    while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
    {
      if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
      {
        DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
        break;
      }

      if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
      {
        DBG_PRINT("error: OWReadByte\r\n");
        DBG_PRINT("error: HAL_I2C_Master_Receive2\r\n");
        Error_Handler();
      }
    }
    status=aRxBuffer[0]& STATUS_1WB;
  } while((status& STATUS_1WB)&& (poll_count++< POLL_LIMIT));

  // check for failure due to poll limit reached
  if(poll_count>= POLL_LIMIT)
  {
    // handle error
    // ...
    //DS2482_reset();

    DBG_PRINT("error: OWReadByte\r\n");
    DBG_PRINT("error: poll limit reached\r\n");

    return 0;
  }

//  I2C_rep_start();
//  I2C_write(I2C_address| I2C_WRITE, EXPECT_ACK);
//  I2C_write(CMD_SRP, EXPECT_ACK);
//  I2C_write(0xE1, EXPECT_ACK);
//  I2C_rep_start();
//  I2C_write(I2C_address| I2C_READ, EXPECT_ACK);
//  data=I2C_read(NACK);
//  I2C_stop();

  aTxBuffer[0]=CMD_SRP;
  aTxBuffer[1]=0xE1;

  while(HAL_I2C_Master_Transmit(&hi2c3, (uint16_t)I2C_ADDRESS, aTxBuffer, 2, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: OWReadByte\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Transmit2\r\n");
      Error_Handler();
    }
  }

  while(HAL_I2C_Master_Receive(&hi2c3, (uint16_t)I2C_ADDRESS, aRxBuffer, 1, 1000)!= HAL_OK)
  {
    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_TIMEOUT)
    {
      DBG_PRINT("error: HAL_I2C_ERROR_TIMEOUT\r\n");
      break;
    }

    if(HAL_I2C_GetError(&hi2c3)!= HAL_I2C_ERROR_AF)
    {
      DBG_PRINT("error: OWReadByte\r\n");
      DBG_PRINT("error: HAL_I2C_Master_Receive3\r\n");
      Error_Handler();
    }
  }
  data=aRxBuffer[0];

  return data;
}

int32_t OW_Copy_Scratchpad(uint8_t ch, uint8_t *array)
{
  if(ch> 1)
  {
    if(use_debug) DBG_PRINT("Channel value is wrong @OW_Copy_Scratchpad\r\n");

    return false;
  }
  else
  {
    uint8_t i, retVal;
    uint8_t tempArr[16]={0, };
    uint8_t encodeValue[16]={0, };
    //uint8_t readArr[16]={0, };
    uint8_t TA1, TA2, E_S;

    // encoding을 하면 값이 변하기 때문에 반드시 원본을 복사해서 사용
    memcpy(tempArr, array, 16);
#if 0
    DBG_PRINT("OriginText: ");
    for(i=0; i< 16; i++)
    {
      DBG_PRINT("%02X ", tempArr[i]);
    }
    DBG_PRINT("\r\n\n");
#endif

    //초기화 벡터값과 공개키로 RoundKey 생성 - 이곳에서 시간 소모가 가장 큼
    AES_init_ctx_iv(&ctx, key, iv);
    // Encryption
    AES_CBC_encrypt_bufferEx(&ctx, tempArr, encodeValue, 16);
#if 0
    DBG_PRINT("CipherText: ");
    for(i=0; i< 16; i++)
    {
      DBG_PRINT("%02X ", encodeValue[i]);
    }
    DBG_PRINT("\r\n\n");
#endif

    DS2482_channel_select(ch);

    OWReset();  // reset pulse & wait presence pulse
    OWWriteByte(0xCC);    // skip rom
    OWWriteByte(0x0F);    // write scratchpad
    OWWriteByte(0x00);    // TA1
    OWWriteByte(0x00);    // TA2
    for(i=0; i< 8; i++)
    {
      OWWriteByte(encodeValue[i]);
    }
    retVal=OWReadByte();  // CRC-16
    retVal=OWReadByte();  // CRC-16

    OWReset();  // reset pulse & wait presence pulse
    OWWriteByte(0xCC);    // skip rom
    OWWriteByte(0xAA);  // read scratchpad
    TA1=OWReadByte();  // TA1
    TA2=OWReadByte();  // TA2
    E_S=OWReadByte(); // E/S byte
    /*for(i=0; i< 8; i++)
     {
     readArr[i]=OWReadByte();
     }
     retVal=OWReadByte();  // CRC-16
     retVal=OWReadByte();  // CRC-16*/

    OWReset();  // reset pulse & wait presence pulse
    OWWriteByte(0xCC);    // skip rom
    OWWriteByte(0x55);    // copy scratchpad
    OWWriteByte(TA1);    // TA1
    OWWriteByte(TA2);    // TA2
    OWWriteByte(E_S);    // E/S byte
    delay_ms(15);
    retVal=OWReadByte();  // read copy status
    if(use_debug) DBG_PRINT("retVal1=%02X\r\n", retVal); //AAh=success

    OWReset();  // reset pulse & wait presence pulse
    OWWriteByte(0xCC);    // skip rom
    OWWriteByte(0x0F);    // write scratchpad
    OWWriteByte(0x08);    // TA1
    OWWriteByte(0x00);    // TA2
    for(i=0; i< 8; i++)
    {
      OWWriteByte(encodeValue[i+ 8]);
    }
    retVal=OWReadByte();  // CRC-16
    retVal=OWReadByte();  // CRC-16
    OWReset();  // reset pulse & wait presence pulse
    OWWriteByte(0xCC);    // skip rom
    OWWriteByte(0xAA);  // read scratchpad
    TA1=OWReadByte();  // TA1
    TA2=OWReadByte();  // TA2
    E_S=OWReadByte(); // E/S byte
    /*for(i=0; i< 8; i++)
     {
     readArr[i+8]=OWReadByte();
     }
     retVal=OWReadByte();  // CRC-16
     retVal=OWReadByte();  // CRC-16*/

    OWReset();  // reset pulse & wait presence pulse
    OWWriteByte(0xCC);    // skip rom
    OWWriteByte(0x55);    // copy scratchpad
    OWWriteByte(TA1);    // TA1
    OWWriteByte(TA2);    // TA2
    OWWriteByte(E_S);    // E/S byte
    delay_ms(15);
    retVal=OWReadByte();  // read copy status
    if(use_debug) DBG_PRINT("retVal2=%02X\r\n", retVal); //AAh=success

    OWReset();  // reset pulse & wait presence pulse
    /*if(use_debug)
     {
     DBG_PRINT("readArr   : ");
     for(i=0; i< 16; i++)
     {
     DBG_PRINT("%02X ", readArr[i]);
     }
     DBG_PRINT("\r\n\n");
     }*/

    return true;
  }
}

uint8_t* OW_Read_Memory(uint8_t ch)
{
  if(ch> 1)
  {
    if(use_debug) DBG_PRINT("Channel value is wrong @OW_Read_Memory\r\n");
    memRead_flag=false;

    return false;
  }
  else
  {
    uint8_t i, empty_count=0;
    static uint8_t arrbuf[16];

    DS2482_channel_select(ch);
    OWReset();  // reset pulse & wait presence pulse
    OWWriteByte(0xCC);  // skip rom
    OWWriteByte(0xF0);  // read memory
    OWWriteByte(0x00);    // TA1
    OWWriteByte(0x00);    // TA2
    for(i=0; i< 16; i++)
    {
      arrbuf[i]=OWReadByte();
      if(arrbuf[i]== 0xFF) empty_count++;
    }
    OWReset();  // reset pulse & wait presence pulse

    if(empty_count> 15)
    {
      if(use_debug) DBG_PRINT("empty_count @OW_Read_Memory\r\n");
      memRead_flag=false;

      return false;
    }
#if 0
    DBG_PRINT("SavedText : ");
    for(i=0; i< 16; i++)
    {
      DBG_PRINT("%02X ", arrbuf[i]);
    }
    DBG_PRINT("\r\n\n");
#endif

    //초기화 벡터값과 공개키로 RoundKey 생성 - 이곳에서 시간 소모가 가장 큼
    AES_init_ctx_iv(&ctx, key, iv);
    // Decryption
    AES_CBC_decrypt_buffer(&ctx, arrbuf, 16);
#if 0
    DBG_PRINT("PlainText : ");
    for(i=0; i< 16; i++)
    {
      DBG_PRINT("%02X ", arrbuf[i]);
    }
    DBG_PRINT("\r\n\n");
#endif

    if(arrbuf[0]!= WATERMARK)
    {
      if(use_debug) DBG_PRINT("WATERMARK @OW_Read_Memory\r\n");
      memRead_flag=false;

      return false;
    }

    return arrbuf;
  }
}

/* End of Code */
