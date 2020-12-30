//-------------------------------------------------------------------------
// Project    : MS19 Project (MainSystem Board)
//-------------------------------------------------------------------------
// MCU        : STM32G474QETx
// CLOCK      : 170MHz
// Date       : 2020.02.21
// Comfiler   : gcc (TBD)
// File Name  : Serial.c
// made by    : dohga
//-------------------------------------------------------------------------

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;

extern TIM_HandleTypeDef htim2;
//extern TIM_HandleTypeDef htim5;

__IO uint8_t tx1_done=1, tx3_done=1, tx5_done=1;

//------------------------------------------------------------------------------
// Serial normal Function
//------------------------------------------------------------------------------
uint8_t BCC_chk(uint8_t *s, uint8_t len)
{
  uint8_t bcc_buf=0, cnt=0;

  while(cnt< len)
  {
    bcc_buf^=s[cnt];
    cnt++;
  }

  return bcc_buf;
}

//------------------------------------------------------------------------------
// UART1 ROUTINE : GUI
//------------------------------------------------------------------------------
void SEND_RESPONSE(void)
{
  tx1_done=0;

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, txlen1);

  /* Enable DMA TX Interrupt */
  LL_USART_EnableDMAReq_TX(USART1);

  /* Enable DMA Channel Tx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

/**
 * @brief  Function called at completion of last byte transmission
 * @param  None
 * @retval None
 */
//void USART1_CharTransmitComplete_Callback()
void DMA1_Channel2_TransmitCplt_Callback()
{
  /* Disable DMA Channel Tx */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

  /* Clear buffer at end of transfer : Tx sequence completed successfully */
  memset(tbuf1, 0, sizeof(tbuf1));
  memset(tempbuf1, 0, sizeof(tempbuf1));

  tx1_done=1;
}

void DLE_encryption1(uint8_t *s, uint32_t len)
{
  uint8_t j=0, i=0;

  while(len--)
  {
    if((s[i]== STX)|| (s[i]== ETX)|| (s[i]== DLE))
    {
      if(i> 1)
      {
        tbuf1[j]=DLE;
        j++;
        tbuf1[j]=s[i]+ 0x40;
      }
      else tbuf1[j]=s[i];
    }
    else tbuf1[j]=s[i];
    i++;
    j++;
  }
  txlen1=j;
}

void SEND_1BYTE(uint8_t cmd, uint8_t data)
{
  uint8_t TL=0;

  buflen1=0;
  tempbuf1[buflen1++]=STX;
  tempbuf1[buflen1++]='g';
  tempbuf1[buflen1++]=_uart.PCount;
  tempbuf1[buflen1++]=0x00;
  tempbuf1[buflen1++]=2;
  tempbuf1[buflen1++]=cmd;
  tempbuf1[buflen1++]=data;

  if(tx1_done)
  {
    DLE_encryption1(tempbuf1, buflen1);
    tbuf1[txlen1++]=ETX;
    TL=txlen1;
    tbuf1[txlen1++]=BCC_chk(tbuf1, TL);
    tx_rp1=0;
    SEND_RESPONSE();
  }
  else DBG_PRINT("tx1 sending...1\r\n");
}

#ifdef USE_1WIRE_MEMORY
void SEND_MEMREPLY(uint8_t data)
{
  uint8_t TL=0;

  buflen1=0;
  tempbuf1[buflen1++]=STX;
  tempbuf1[buflen1++]='g';
  tempbuf1[buflen1++]=_uart.PCount;
  tempbuf1[buflen1++]=0x00;
  tempbuf1[buflen1++]=4;
  tempbuf1[buflen1++]=CMD_MEMORY;
  tempbuf1[buflen1++]=_uart.reply;
  tempbuf1[buflen1++]=_memory.loc;
  tempbuf1[buflen1++]=data;

  if(tx1_done)
  {
    DLE_encryption1(tempbuf1, buflen1);
    tbuf1[txlen1++]=ETX;
    TL=txlen1;
    tbuf1[txlen1++]=BCC_chk(tbuf1, TL);
    tx_rp1=0;
    SEND_RESPONSE();
  }
  else DBG_PRINT("tx1 sending...2\r\n");
}

void SEND_MEMREAD(uint8_t *readVal)
{
  uint8_t TL=0;

  buflen1=0;
  tempbuf1[buflen1++]=STX;
  tempbuf1[buflen1++]='g';
  tempbuf1[buflen1++]=_uart.PCount;
  tempbuf1[buflen1++]=0x00;
  tempbuf1[buflen1++]=7+ _memory.len;
  tempbuf1[buflen1++]=CMD_MEMORY;
  tempbuf1[buflen1++]=_uart.reply;
  tempbuf1[buflen1++]=_memory.loc;
  tempbuf1[buflen1++]=2;
  tempbuf1[buflen1++]=readVal[1];
  tempbuf1[buflen1++]=readVal[2];
  tempbuf1[buflen1++]=readVal[3];
  for(int i=0; i< _memory.len; i++)
  {
    tempbuf1[buflen1++]=readVal[i+ 4];
  }

  if(tx1_done)
  {
    DLE_encryption1(tempbuf1, buflen1);
    tbuf1[txlen1++]=ETX;
    TL=txlen1;
    tbuf1[txlen1++]=BCC_chk(tbuf1, TL);
    tx_rp1=0;
    SEND_RESPONSE();
  }
  else DBG_PRINT("tx1 sending...3\r\n");
}
#endif

void SEND_TEMP(uint16_t D01, uint16_t D23, uint8_t D4)
{
  uint8_t TL=0;

  buflen1=0;
  tempbuf1[buflen1++]=STX;
  tempbuf1[buflen1++]='g';
  tempbuf1[buflen1++]=_uart.PCount;
  tempbuf1[buflen1++]=0x00;
  tempbuf1[buflen1++]=7;
  tempbuf1[buflen1++]=CMD_TEMPER;
  tempbuf1[buflen1++]=_uart.reply;
  tempbuf1[buflen1++]=D01& 0xff;
  tempbuf1[buflen1++]=D01>> 8;
  tempbuf1[buflen1++]=D23& 0xff;
  tempbuf1[buflen1++]=D23>> 8;
  tempbuf1[buflen1++]=D4;

  if(tx1_done)
  {
    DLE_encryption1(tempbuf1, buflen1);
    tbuf1[txlen1++]=ETX;
    TL=txlen1;
    tbuf1[txlen1++]=BCC_chk(tbuf1, TL);
    tx_rp1=0;
    SEND_RESPONSE();
  }
  else DBG_PRINT("tx1 sending...4\r\n");
}

void SEND_HALL(uint8_t D0, uint16_t D12, uint16_t D34)
{
  uint8_t TL=0;

  buflen1=0;
  tempbuf1[buflen1++]=STX;
  tempbuf1[buflen1++]='g';
  tempbuf1[buflen1++]=_uart.PCount;
  tempbuf1[buflen1++]=0x00;
  tempbuf1[buflen1++]=6;
  tempbuf1[buflen1++]=CMD_HALL;
  tempbuf1[buflen1++]=D0;
  tempbuf1[buflen1++]=D12& 0xff;
  tempbuf1[buflen1++]=D12>> 8;
  tempbuf1[buflen1++]=D34& 0xff;
  tempbuf1[buflen1++]=D34>> 8;

  if(tx1_done)
  {
    DLE_encryption1(tempbuf1, buflen1);
    tbuf1[txlen1++]=ETX;
    TL=txlen1;
    tbuf1[txlen1++]=BCC_chk(tbuf1, TL);
    tx_rp1=0;
    SEND_RESPONSE();
  }
  else DBG_PRINT("tx1 sending...5\r\n");
}

void SEND_VERSION()
{
  uint8_t TL=0;
  uint16_t ver1=SystemVersion, ver2=PSC_Version1, ver3=PSC_Version2;

  buflen1=0;
  tempbuf1[buflen1++]=STX;
  tempbuf1[buflen1++]='g';
  tempbuf1[buflen1++]=_uart.PCount;
  tempbuf1[buflen1++]=0x00;
  tempbuf1[buflen1++]=8;
  tempbuf1[buflen1++]=CMD_VERSION;
  tempbuf1[buflen1++]=_uart.reply;
  tempbuf1[buflen1++]=ver1& 0xff;
  tempbuf1[buflen1++]=ver1>> 8;
  tempbuf1[buflen1++]=ver2& 0xff;
  tempbuf1[buflen1++]=ver2>> 8;
  tempbuf1[buflen1++]=ver3& 0xff;
  tempbuf1[buflen1++]=ver3>> 8;

  if(tx1_done)
  {
    DLE_encryption1(tempbuf1, buflen1);
    tbuf1[txlen1++]=ETX;
    TL=txlen1;
    tbuf1[txlen1++]=BCC_chk(tbuf1, TL);
    tx_rp1=0;
    SEND_RESPONSE();
  }
  else DBG_PRINT("tx1 sending...6\r\n");
}

void SEND_STATUS()
{
  uint8_t TL=0;

  buflen1=0;
  tempbuf1[buflen1++]= STX;
  tempbuf1[buflen1++]='g';
  tempbuf1[buflen1++]=_uart.PCount;
  tempbuf1[buflen1++]=0x00;
  tempbuf1[buflen1++]=5;
  tempbuf1[buflen1++]=CMD_STATUS;
  tempbuf1[buflen1++]=SF1.STATUS1B;
  tempbuf1[buflen1++]=SF2.STATUS2B;
  tempbuf1[buflen1++]=SF3.STATUS3B;
  tempbuf1[buflen1++]=SF4.STATUS4B;

  if(tx1_done)
  {
    DLE_encryption1(tempbuf1, buflen1);
    tbuf1[txlen1++]=ETX;
    TL=txlen1;
    tbuf1[txlen1++]=BCC_chk(tbuf1, TL);
    tx_rp1=0;
    SEND_RESPONSE();
  }
  else DBG_PRINT("tx1 sending...7\r\n");
}

void SEND_ERROR()
{
  uint8_t TL=0;

  buflen1=0;
  tempbuf1[buflen1++]= STX;
  tempbuf1[buflen1++]='g';
  tempbuf1[buflen1++]=_uart.PCount;
  tempbuf1[buflen1++]=0x00;
  tempbuf1[buflen1++]=5;
  tempbuf1[buflen1++]=CMD_ERROR;
  tempbuf1[buflen1++]=EF1.ERROR1B;
  tempbuf1[buflen1++]=EF2.ERROR2B;
  tempbuf1[buflen1++]=EF3.ERROR3B;
  tempbuf1[buflen1++]=EF4.ERROR4B;

  if(tx1_done)
  {
    DLE_encryption1(tempbuf1, buflen1);
    tbuf1[txlen1++]=ETX;
    TL=txlen1;
    tbuf1[txlen1++]=BCC_chk(tbuf1, TL);
    tx_rp1=0;
    SEND_RESPONSE();
  }
  else DBG_PRINT("tx1 sending...8\r\n");
}

/**
 * @brief  Function called from DMA1 IRQ Handler when Rx transfer is completed
 * @param  None
 * @retval None
 */
void DMA1_Channel1_ReceiveCplt_Callback()
{
  uint8_t rxbuf=DMA_USART1_buf[0];
  static uint8_t dle_flag1=0, etx_flag1=0;

  bcc_check_buffer1=rxbuf;

  if(rxlen1< rx_buff_size1)
  {
    if(rxlen1== 0)
    {
      if((rxbuf!= STX)&& (rxbuf!= ACK)&& (rxbuf!= NAK))
      {
        memset(rbuf1, 0, sizeof(rbuf1));

        return;
      }
    }

    if(etx_flag1)
    {
      etx_flag1=RESET;

      if(bccchk1!= bcc_check_buffer1) _bit_txnak1=SET;
      else _bit_txnak1=RESET;

      CHK_CMD();
    }
    else
    {
      if((rxbuf== NAK)&& (rxlen1== 0))
      {
        if(GUI_CMD) _bit_rxnak1=SET;
      }
      else
      {
        if(rxbuf== STX) rxlen1=0;

        if(rxlen1== 0) bccchk1=rxbuf;
        else bccchk1^=rxbuf;

        if(rxbuf== ETX) etx_flag1=SET;

        if(dle_flag1)
        {
          dle_flag1=RESET;
          rbuf1[rxlen1++]=rxbuf- 0x40;
        }
        else
        {
          if(rxbuf== DLE) dle_flag1=SET;
          else rbuf1[rxlen1++]=rxbuf;
        }
      }
    }
  }
}

void CHK_CMD(void)
{
  if(_bit_txnak1== SET)
  {
    LL_USART_TransmitData8(USART1, NAK);
    _bit_txnak1=RESET;
  }
  else
  {
    uint8_t i, ChkBf[rx_buff_size1];

    for(i=0; i< rxlen1; i++)
    {
      ChkBf[i]=rbuf1[i];
    }
    _uart.PCount=ChkBf[2];
    _uart.reply=ChkBf[6];

    switch(ChkBf[5])
    {
      case 'E':
        //GUI_Response=1;
        sendError=0;
        break;

      case 'L':
        if(ChkBf[6]== 0)  // Front LED
        {
          if(ChkBf[7]== 0)
          {
            FRONT_LED_OFF;
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
            TIM2->CCR1=0;
          }
          else
          {
            FRONT_LED_ON;
            TIM2->CCR1=5000* ChkBf[7]/ 100;
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
          }
        }
        else if(ChkBf[6]== 1) // Jog Backlight
        {
          if(ChkBf[7]== 0)
          {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            TIM2->CCR2=0;
          }
          else
          {
            TIM2->CCR2=5000* ChkBf[7]/ 100;
            HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
          }
        }
        else if(ChkBf[6]== 2) // AP A LED
        {
          if(ChkBf[7]== 0)
          {
            LL_GPIO_ResetOutputPin(AP_A_LED1_GPIO_Port, AP_A_LED1_Pin);
            LL_GPIO_ResetOutputPin(AP_A_LED2_GPIO_Port, AP_A_LED2_Pin);
            LL_GPIO_ResetOutputPin(AP_A_LED3_GPIO_Port, AP_A_LED3_Pin);
          }
          else
          {
            LL_GPIO_SetOutputPin(AP_A_LED1_GPIO_Port, AP_A_LED1_Pin);
            LL_GPIO_SetOutputPin(AP_A_LED2_GPIO_Port, AP_A_LED2_Pin);
            LL_GPIO_SetOutputPin(AP_A_LED3_GPIO_Port, AP_A_LED3_Pin);
          }
        }
        else if(ChkBf[6]== 3) // AP B LED
        {
          if(ChkBf[7]== 0)
          {
            LL_GPIO_ResetOutputPin(AP_B_LED1_GPIO_Port, AP_B_LED1_Pin);
            LL_GPIO_ResetOutputPin(AP_B_LED2_GPIO_Port, AP_B_LED2_Pin);
            LL_GPIO_ResetOutputPin(AP_B_LED3_GPIO_Port, AP_B_LED3_Pin);
          }
          else
          {
            LL_GPIO_SetOutputPin(AP_B_LED1_GPIO_Port, AP_B_LED1_Pin);
            LL_GPIO_SetOutputPin(AP_B_LED2_GPIO_Port, AP_B_LED2_Pin);
            LL_GPIO_SetOutputPin(AP_B_LED3_GPIO_Port, AP_B_LED3_Pin);
          }
        }
        else if(ChkBf[6]== 4) // AP A Fan
        {
          if(ChkBf[7]== 0)
          {
            AP_A_FAN_OFF();
            //AP_A_FAN_STOP;
          }
          else
          {
            AP_A_FAN_ON();
            //AP_A_FAN_START;
          }
        }
        else if(ChkBf[6]== 5) // AP B Fan
        {
          if(ChkBf[7]== 0)
          {
            AP_B_FAN_OFF();
            //AP_B_FAN_STOP;
          }
          else
          {
            AP_B_FAN_ON();
            //AP_B_FAN_START;
          }
        }
        SEND_1BYTE('L', _uart.reply);
        break;

#ifdef USE_1WIRE_MEMORY
      case 'M':
        if(ChkBf[6]== 0)
        {
          if(mem_done)
          {
            //DBG_PRINT("Memory Write request\r\n");
            mem_done=0;
            _memory.loc=ChkBf[7];
            _memory.ch=ChkBf[8];
            _memory.id=ChkBf[9];
            _memory.len=ChkBf[10];
            if(_memory.len> 10) _memory.len=10;

            for(int i=0; i< _memory.len; i++)
            {
              _memory.asc[i]=ChkBf[i+ 11];
            }

            GUI_CMD=CMD_MEMORY;
          }
        }
        else if(ChkBf[6]== 1)
        {
          if(mem_done)
          {
            //DBG_PRINT("Memory Read request\r\n");
            mem_done=0;
            _memory.loc=ChkBf[7];
            memRead_cnt=0;

            GUI_CMD=CMD_MEMORY;
          }
        }
        break;
#endif

      case 'O':
        GUI_CMD=CMD_GUI_ONOFF;
        break;

      case 'P':
        GUI_CMD=CMD_PARAMETER;

        if(ChkBf[6]== 0)
        {
          if(pmr.mode!= ChkBf[7])
          {
            pmr.mode=ChkBf[7];

            if(use_debug)
            {
              if(pmr.mode== TWIST) DBG_PRINT("-- TWIST Mode --\r\n");
              else if(pmr.mode== HOLD) DBG_PRINT("-- HOLD Mode --\r\n");
              else if(pmr.mode== GRIP) DBG_PRINT("-- GRIP Mode --\r\n");
              else if(pmr.mode== TAP) DBG_PRINT("-- TAP Mode --\r\n");
              else if(pmr.mode== SETUP) DBG_PRINT("-- SETUP Mode --\r\n");
            }

            /*if(!SF1.bStatus11&& (CH_LEFT== RUN_MODE))
             {
             phase_1A_flag=0;
             phase_1A_count=0;
             phase_2A_flag=0;
             phase_2A_count=0;
             phase_3A_flag=0;
             phase_3A_count=0;
             phase_4A_flag=0;
             phase_4A_count=0;
             Ready_A_Stop();
             chA_DA=CONSTANT_DA;
             POWER_DA_CONTROL_A(chA_DA);
             CH_LEFT=CHANGE_MODE;
             }

             if(!SF1.bStatus12&& (CH_RIGHT== RUN_MODE))
             {
             phase_1B_flag=0;
             phase_1B_count=0;
             phase_2B_flag=0;
             phase_2B_count=0;
             phase_3B_flag=0;
             phase_3B_count=0;
             phase_4B_flag=0;
             phase_4B_count=0;
             Ready_B_Stop();
             chB_DA=CONSTANT_DA;
             POWER_DA_CONTROL_B(chB_DA);
             CH_RIGHT=CHANGE_MODE;
             }*/

            if(pmr.freq0!= ChkBf[8])
            {
              pmr.freq0=ChkBf[8];
              Q_change_A(pmr.freq0);
              Q_change_B(pmr.freq0);
              if(use_debug) DBG_PRINT("Frequency0 is changed\r\n");
            }

            pmr.freq1=ChkBf[9];
            pmr.freq2=ChkBf[10];
            pmr.freq3=ChkBf[11];
            pmr.freq4=ChkBf[12];
            pmr.freq5=ChkBf[13];

            if(use_debug)
            {
              DBG_PRINT("Frequency0=%d\r\n", ChkBf[8]);
              DBG_PRINT("Frequency1=%d\r\n", ChkBf[9]);
              DBG_PRINT("Frequency2=%d\r\n", ChkBf[10]);
              DBG_PRINT("Frequency3=%d\r\n", ChkBf[11]);
              DBG_PRINT("Frequency4=%d\r\n", ChkBf[12]);
              DBG_PRINT("Frequency5=%d\r\n\n", ChkBf[3]);
            }
          }
        }
        else if(ChkBf[6]== 1)
        {
          pmr.uTime=ChkBf[7];
          pmr.s1=ChkBf[8];
          pmr.s2=ChkBf[9];
          pmr.s3=ChkBf[10];
          pmr.s4=ChkBf[11];
          pmr.s5=ChkBf[12];
          pmr.sTime=(ChkBf[8]+ ChkBf[9]+ ChkBf[10]+ ChkBf[11]+ ChkBf[12]);
          pmr.dTime=ChkBf[13];
          pmr.oTime=ChkBf[14];

          if(use_debug)
          {
            DBG_PRINT("U=%d S=%d D=%d O=%d\r\n", pmr.uTime, pmr.sTime, pmr.dTime, pmr.oTime);
            DBG_PRINT("s1=%d s2=%d s3=%d s4=%d s5=%d\r\n", pmr.s1, pmr.s2, pmr.s3, pmr.s4, pmr.s5);
          }
        }
        //TODO
        else if(ChkBf[6]== 2)
        {
          pmr.intenCh=ChkBf[7];

          if(pmr.intenCh== 0)
          {
            if((ChkBf[8]== 0)&& (ChkBf[9]== 0))
            {
//              if(operation_flag== 2)
//              {
              pmr.intenA_new=0;
              pmr.linked=0;

              if((CH_LEFT== RUN_MODE)|| (CH_LEFT== CHANGE_MODE)|| (CH_LEFT== REST_MODE))
              {
                CH_LEFT=PAUSE_MODE;
                if(use_debug) DBG_PRINT("CH_LEFT=PAUSE_MODE\r\n");

                dischargeA=1;
                if(phase_4A_flag== 1) RunToPause(CHANNEL_L);
                else if((_master_flag== CHANNEL_R)&& _twist_once_) RunToPause(CHANNEL_L);
                else
                {
                  intenA_Zero=1;
                  dStep5=pmr.freq0;
                  chA_dStep5=(pmr.intenA/ dStep5)+ 1;
                  pmr.intenA=0;

                  phase_1A_flag=0;
                  phase_2A_flag=0;
                  phase_3A_flag=0;
                  phase_4A_flag=0;

                  phase_5A_count=0;
                  phase_5A_flag=1;
                }
              }

              if(_master_flag== CHANNEL_L)
              {
                if(!SF1.bStatus12&& ((CH_RIGHT== RUN_MODE)|| (CH_RIGHT== CHANGE_MODE)))
                {
                  _master_flag=CHANNEL_R;
                  if((pmr.mode== TWIST)&& _twist_once_)
                  {
                    //CH_RIGHT=CHANGE_MODE;
                    _twist_once_=0;
                    Ready_B_Start();
                  }
                  else if(_next_step_)
                  {
                    _next_step_=0;
                    Ready_B_Start();
                  }
                }
              }
              else if(_master_flag== CHANNEL_R)
              {
                if(!SF1.bStatus12&& ((CH_RIGHT== RUN_MODE)|| (CH_RIGHT== CHANGE_MODE)))
                {
                  if(pending_flag)
                  {
                    pending_flag=0;
                    Ready_B_Start();
                  }
                }
              }
//                if(use_debug) DBG_PRINT("ch_L: Intensity is zero\r\n");
//              }
              SEND_1BYTE(CMD_PARAMETER, _uart.reply);
              GUI_CMD=RESET;
            }
            else
            {
              if(!dischargeA)
              {
                intenA_reply_cnt=0;
                pmr.intenA_new=byte2int(ChkBf[9], ChkBf[8]);
                if(pmr.intenA_new< CONSTANT_DA) pmr.intenA_new=CONSTANT_DA;
                else if(pmr.intenA_new> 4095) pmr.intenA_new=4095;
                if(use_debug) DBG_PRINT("ch_L: Intensity=%ld\r\n", pmr.intenA_new);

                if(CH_LEFT== REST_MODE)
                {
                  CH_LEFT=CHANGE_MODE;
                  if(use_debug) DBG_PRINT("CH_LEFT=CHANGE_MODE\r\n");
                }
              }
              else
              {
                intenA_reply_cnt++;
                if(intenA_reply_cnt> 2)
                {
                  intenA_reply_cnt=0;
                  dischargeA=0;
                  SEND_1BYTE(CMD_PARAMETER, _uart.reply);
                  GUI_CMD=RESET;
                }
                else GUI_CMD=RESET;
              }
            }

            if(CH_LEFT== RUN_MODE) //GUI_CMD=cmd_reaction1;
            {
              reaction_A_flag=1;
              SEND_1BYTE(CMD_PARAMETER, _uart.reply);
              GUI_CMD=RESET;
            }
          }
          else if(pmr.intenCh== 1)
          {
            if((ChkBf[8]== 0)&& (ChkBf[9]== 0))
            {
//              if(operation_flag== 2)
//              {
              pmr.intenB_new=0;
              pmr.linked=0;

              if((CH_RIGHT== RUN_MODE)|| (CH_RIGHT== CHANGE_MODE)|| (CH_RIGHT== REST_MODE))
              {
                CH_RIGHT=PAUSE_MODE;
                if(use_debug) DBG_PRINT("CH_RIGHT=PAUSE_MODE\r\n");

                dischargeB=1;
                if(phase_4B_flag== 1) RunToPause(CHANNEL_R);
                else if((_master_flag== CHANNEL_L)&& _twist_once_) RunToPause(CHANNEL_R);
                else
                {
                  intenB_Zero=1;
                  dStep5=pmr.freq0;
                  chB_dStep5=(pmr.intenB/ dStep5)+ 1;
                  pmr.intenB=0;

                  phase_1B_flag=0;
                  phase_2B_flag=0;
                  phase_3B_flag=0;
                  phase_4B_flag=0;

                  phase_5B_count=0;
                  phase_5B_flag=1;
                }
              }

              if(_master_flag== CHANNEL_R)
              {
                if(!SF1.bStatus11&& ((CH_LEFT== RUN_MODE)|| (CH_LEFT== CHANGE_MODE)))
                {
                  _master_flag=CHANNEL_L;
                  if((pmr.mode== TWIST)&& _twist_once_)
                  {
                    //CH_LEFT=CHANGE_MODE;
                    _twist_once_=0;
                    Ready_A_Start();
                  }
                  else if(_next_step_)
                  {
                    _next_step_=0;
                    Ready_A_Start();
                  }
                }
              }
              else if(_master_flag== CHANNEL_L)
              {
                if(!SF1.bStatus11&& ((CH_LEFT== RUN_MODE)|| (CH_LEFT== CHANGE_MODE)))
                {
                  if(pending_flag)
                  {
                    pending_flag=0;
                    Ready_A_Start();
                  }
                }
              }
//                if(use_debug) DBG_PRINT("ch_R: Intensity is zero\r\n");
//              }
              SEND_1BYTE(CMD_PARAMETER, _uart.reply);
              GUI_CMD=RESET;
            }
            else
            {
              if(!dischargeB)
              {
                intenB_reply_cnt=0;
                pmr.intenB_new=byte2int(ChkBf[9], ChkBf[8]);
                if(pmr.intenB_new< CONSTANT_DA) pmr.intenB_new=CONSTANT_DA;
                else if(pmr.intenB_new> 4095) pmr.intenB_new=4095;
                if(use_debug) DBG_PRINT("ch_R: Intensity=%ld\r\n", pmr.intenB_new);

                if(CH_RIGHT== REST_MODE)
                {
                  CH_RIGHT=CHANGE_MODE;
                  if(use_debug) DBG_PRINT("CH_RIGHT=CHANGE_MODE\r\n");
                }
              }
              else
              {
                intenB_reply_cnt++;
                if(intenB_reply_cnt> 2)
                {
                  intenB_reply_cnt=0;
                  dischargeB=0;
                  SEND_1BYTE(CMD_PARAMETER, _uart.reply);
                  GUI_CMD=RESET;
                }
                else GUI_CMD=RESET;
              }
            }

            if(CH_RIGHT== RUN_MODE) //GUI_CMD=cmd_reaction2;
            {
              reaction_B_flag=1;
              SEND_1BYTE(CMD_PARAMETER, _uart.reply);
              GUI_CMD=RESET;
            }
          }
          pmr.linked=ChkBf[10];
        }
        else if(ChkBf[6]== 3)
        {
          if(use_debug)
          {
            if(ChkBf[7]) DBG_PRINT("ch_L is ON\r\n");
            else DBG_PRINT("ch_L is OFF\r\n");

            if(ChkBf[8]) DBG_PRINT("ch_R is ON\r\n");
            else DBG_PRINT("ch_R is OFF\r\n");
          }
        }
        break;

        //TODO
      case 'R':
        GUI_CMD=CMD_RUN;
        if(use_debug) DBG_PRINT("\n");

        if((ChkBf[6]== 0)|| (ChkBf[6]== 1))
        {
          if(use_debug) DBG_PRINT("recive: Stop\r\n");

          old_CH_L=CH_LEFT;
          CH_LEFT=STOP_MODE;
          if(use_debug) DBG_PRINT("CH_LEFT=STOP_MODE\r\n");

          old_CH_R=CH_RIGHT;
          CH_RIGHT=STOP_MODE;
          if(use_debug) DBG_PRINT("CH_RIGHT=STOP_MODE\r\n");

          //if((operation_flag== 1)|| (operation_flag== 3))
          if(operation_flag== 3)
          {
            operation_flag=0;
            dischargeA=1;
            RunToStopA();
            dischargeB=1;
            RunToStopB();
          }
          else
          {
            operation_flag=0;

            dStep5=pmr.freq0;
            if(old_CH_L!= STOP_MODE)
            {
              dischargeA=1;
              if((old_CH_L== PAUSE_MODE)|| (old_CH_L== REST_MODE)) RunToStopA();
              else if(phase_4A_flag== 1) RunToStopA();
              else if((_master_flag== CHANNEL_R)&& _twist_once_) RunToStopA();
              else
              {
                chA_dStep5=(pmr.intenA/ dStep5)+ 1;
//                if(use_debug) DBG_PRINT("STOP chA_dStep5=%ld\r\n", chA_dStep5);

                phase_1A_flag=0;
                phase_2A_flag=0;
                phase_3A_flag=0;
                phase_4A_flag=0;

                phase_5A_count=0;
                phase_5A_flag=1;
              }
            }

            if(old_CH_R!= STOP_MODE)
            {
              dischargeB=1;
              if((old_CH_R== PAUSE_MODE)|| (old_CH_R== REST_MODE)) RunToStopB();
              else if(phase_4B_flag== 1) RunToStopB();
              else if((_master_flag== CHANNEL_L)&& _twist_once_) RunToStopB();
              else
              {
                chB_dStep5=(pmr.intenB/ dStep5)+ 1;
//                if(use_debug) DBG_PRINT("STOP chB_dStep=%ld\r\n", chB_dStep);

                phase_1B_flag=0;
                phase_2B_flag=0;
                phase_3B_flag=0;
                phase_4B_flag=0;

                phase_5B_count=0;
                phase_5B_flag=1;
              }
            }
          }

          pmr.uTime=0;
          pmr.s1=0;
          pmr.s2=0;
          pmr.s3=0;
          pmr.s4=0;
          pmr.s5=0;
          pmr.sTime=0;
          pmr.dTime=0;
          pmr.oTime=0;
          pmr.intenA=0;
          pmr.intenA_new=0;
          pmr.intenB=0;
          pmr.intenB_new=0;
          RunToStopVal();
        }
//        else if(ChkBf[6]== 1)
//        {
//          if(use_debug) DBG_PRINT("recive: Pause\r\n");
//
//          operation_flag=1;
//          dStep=pmr.freq0;
//
//          if((CH_LEFT!= PAUSE_MODE)&& (CH_LEFT!= STOP_MODE))
//          {
//            if(phase_4A_flag== 1) RunToPause(CHANNEL_L);
//            else if((_master_flag== CHANNEL_R)&& _twist_once_) RunToPause(CHANNEL_L);
//            else
//            {
//              chA_dStep=(pmr.intenA/ dStep)+ 1;
////              if(use_debug) DBG_PRINT("PAUSE chA_dStep=%ld\r\n", chA_dStep);
//              pmr.intenA=0;
//
//              phase_1A_flag=0;
//              phase_2A_flag=0;
//              phase_4A_flag=0;
//              phase_3A_count=0;
//              phase_3A_flag=1;
//            }
//          }
//
//          if((CH_RIGHT!= PAUSE_MODE)&& (CH_RIGHT!= STOP_MODE))
//          {
//            if(phase_4B_flag== 1) RunToPause(CHANNEL_R);
//            else if((_master_flag== CHANNEL_L)&& _twist_once_) RunToPause(CHANNEL_R);
//            else
//            {
//              chB_dStep=(pmr.intenB/ dStep)+ 1;
////              if(use_debug) DBG_PRINT("PAUSE chB_dStep=%ld\r\n", chB_dStep);
//              pmr.intenB=0;
//
//              phase_1B_flag=0;
//              phase_2B_flag=0;
//              phase_4B_flag=0;
//              phase_3B_count=0;
//              phase_3B_flag=1;
//            }
//          }
//
//          pmr.uTime=0;
//          pmr.s1=0;
//          pmr.s2=0;
//          pmr.s3=0;
//          pmr.s4=0;
//          pmr.s5=0;
//          pmr.sTime=0;
//          pmr.dTime=0;
//          pmr.oTime=0;
//        }
        else if(ChkBf[6]== 2)
        {
          if(use_debug) DBG_PRINT("recive: Run\r\n");

          /* Start channel 1 (Q-pulse-A) */
          if(!SF1.bStatus11)
          {
            Q_change_A(pmr.freq0);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
          }

          /* Start channel 1 (Q-pulse-B) */
          if(!SF1.bStatus12)
          {
            Q_change_B(pmr.freq0);
            HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
          }

          operation_flag=2;
        }
        else if(ChkBf[6]== 3)
        {
          if(use_debug) DBG_PRINT("recive: Rest\r\n");

          operation_flag=3;
          dStep5=pmr.freq0;

          //if(((CH_LEFT!= REST_MODE)&& (CH_LEFT!= PAUSE_MODE)&& (CH_LEFT!= STOP_MODE))|| ((CH_LEFT== PAUSE_MODE)&& (pmr.intenA_new!= 0)))
          if((CH_LEFT!= REST_MODE)&& (CH_LEFT!= STOP_MODE)&& (pmr.intenA_new!= 0))
          {
            if((old_CH_L== PAUSE_MODE)&& (CH_LEFT== RUN_MODE)&& _next_step_) chA_DA_zero=1;

            old_CH_L=CH_LEFT;
            CH_LEFT=REST_MODE;
            if(use_debug) DBG_PRINT("CH_LEFT=REST_MODE\r\n");

            if(old_CH_L== PAUSE_MODE) RunToRest(CHANNEL_L);
            else if(phase_4A_flag== 1) RunToRest(CHANNEL_L);
            else if((_master_flag== CHANNEL_R)&& (old_CH_L== RUN_MODE)&& _next_step_)
            {
              _next_step_=0;
              RunToRest(CHANNEL_L);
            }
            else
            {
              if(pmr.intenA> CONSTANT_DA) chA_dStep5=((pmr.intenA- CONSTANT_DA)/ dStep5)+ 1;
              else chA_dStep5=0;
//              if(use_debug) DBG_PRINT("REST chA_dStep=%ld\r\n", chA_dStep);

              if(old_CH_L== PAUSE_MODE) pmr.intenA=0;
              else pmr.intenA=CONSTANT_DA;

              phase_1A_flag=0;
              phase_2A_flag=0;
              phase_3A_flag=0;
              phase_4A_flag=0;

              phase_5A_count=0;
              phase_5A_flag=1;
            }
          }

          //if(((CH_RIGHT!= REST_MODE)&& (CH_RIGHT!= PAUSE_MODE)&& (CH_RIGHT!= STOP_MODE))|| ((CH_RIGHT== PAUSE_MODE)&& (pmr.intenB_new!= 0)))
          if((CH_RIGHT!= REST_MODE)&& (CH_RIGHT!= STOP_MODE)&& (pmr.intenB_new!= 0))
          {
            if((old_CH_R== PAUSE_MODE)&& (CH_RIGHT== RUN_MODE)&& _next_step_) chB_DA_zero=1;

            old_CH_R=CH_RIGHT;
            CH_RIGHT=REST_MODE;
            if(use_debug) DBG_PRINT("CH_RIGHT=REST_MODE\r\n");

            if(old_CH_R== PAUSE_MODE) RunToRest(CHANNEL_R);
            else if(phase_4B_flag== 1) RunToRest(CHANNEL_R);
            else if((_master_flag== CHANNEL_L)&& (old_CH_R== RUN_MODE)&& _next_step_)
            {
              _next_step_=0;
              RunToRest(CHANNEL_R);
            }
            else
            {
              if(pmr.intenB> CONSTANT_DA) chB_dStep5=((pmr.intenB- CONSTANT_DA)/ dStep5)+ 1;
              else chB_dStep5=0;
//              if(use_debug) DBG_PRINT("REST chB_dStep=%ld\r\n", chB_dStep);

              if(old_CH_R== PAUSE_MODE) pmr.intenB=0;
              else pmr.intenB=CONSTANT_DA;

              phase_1B_flag=0;
              phase_2B_flag=0;
              phase_3B_flag=0;
              phase_4B_flag=0;

              phase_5B_count=0;
              phase_5B_flag=1;
            }
          }

          pmr.uTime=0;
          pmr.s1=0;
          pmr.s2=0;
          pmr.s3=0;
          pmr.s4=0;
          pmr.s5=0;
          pmr.sTime=0;
          pmr.dTime=0;
          pmr.oTime=0;
        }

        SEND_1BYTE(CMD_RUN, _uart.reply);
        break;

      case 'S':
        sendStatus=0;
        break;

      case 'T':
        GUI_CMD=CMD_TEMPER;
        if(countTemp> 0) countTemp--;
        else if(countTemp== 0) commErr=RESET;
        break;

      case 'V':
        GUI_CMD=CMD_VERSION;
        ver_count++;
        if(ver_count> 2)
        {
          PSC_Version1=999;
          PSC_Version2=999;
          SEND_VERSION();
          ver_count=0;
          PSC_Version1=0;
          PSC_Version2=0;
          if(use_debug) DBG_PRINT("reply VERSION, but PSC not normal\r\n");

          if(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_4))
          {
            if(PSC_VerCheck1== 1) EF1.bError17=1;
            if(PSC_VerCheck2== 1) EF1.bError18=1;
          }
        }
        break;

      default:
        GUI_CMD=CMD_undefine;
        break;
    }
  }
}

//------------------------------------------------------------------------------
// UART2 ROUTINE : PSC 1
//------------------------------------------------------------------------------
void _send_response3(void)
{
  tx3_done=0;

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, txlen3);

  /* Enable DMA TX Interrupt */
  LL_USART_EnableDMAReq_TX(USART3);

  /* Enable DMA Channel Tx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
}

/**
 * @brief  Function called at completion of last byte transmission
 * @param  None
 * @retval None
 */
void DMA1_Channel4_TransmitCplt_Callback()
{
  /* Disable DMA Channel Tx */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);

  /* Clear buffer at end of transfer : Tx sequence completed successfully */
  memset(tbuf3, 0, sizeof(tbuf3));
  memset(tempbuf3, 0, sizeof(tempbuf3));

  tx3_done=1;
}

void DLE_encryption3(uint8_t *s, uint32_t len)
{
  uint8_t j=0, i=0;

  while(len--)
  {
    if((s[i]== STX)|| (s[i]== ETX)|| (s[i]== DLE))
    {
      if(i> 1)
      {
        tbuf3[j]=DLE;
        j++;
        tbuf3[j]=s[i]+ 0x40;
      }
      else tbuf3[j]=s[i];
    }
    else tbuf3[j]=s[i];
    i++;
    j++;
  }

  txlen3=j;
}

void _send_cmd3(uint8_t name)
{
  uint8_t TL=0;

  buflen3=0;
  tempbuf3[buflen3++]=STX;
  tempbuf3[buflen3++]='p';
  tempbuf3[buflen3++]=0x01;
  tempbuf3[buflen3++]=name;

  if(tx3_done)
  {
    DLE_encryption3(tempbuf3, buflen3);
    tbuf3[txlen3++]=ETX;
    TL=txlen3;
    tbuf3[txlen3++]=BCC_chk(tbuf3, TL);
    tx_rp3=0;
    _send_response3();
  }
  else DBG_PRINT("tx3 sending...1\r\n");
}

void _send_1byte3(uint8_t name, uint8_t _byte)
{
  uint8_t TL=0;

  buflen3=0;
  tempbuf3[buflen3++]=STX;
  tempbuf3[buflen3++]='p';
  tempbuf3[buflen3++]=0x02;
  tempbuf3[buflen3++]=name;
  tempbuf3[buflen3++]=_byte;

  if(tx3_done)
  {
    DLE_encryption3(tempbuf3, buflen3);
    tbuf3[txlen3++]=ETX;
    TL=txlen3;
    tbuf3[txlen3++]=BCC_chk(tbuf3, TL);
    tx_rp3=0;
    _send_response3();
  }
  else DBG_PRINT("tx3 sending...2\r\n");
}

void DMA1_Channel3_ReceiveCplt_Callback()
{
  uint8_t rxbuf=DMA_USART3_buf[0];
  static uint8_t dle_flag3=0, etx_flag3=0;

  bcc_check_buffer3=rxbuf;

  if(rxlen3< rx_buff_size3)
  {
    if(rxlen3== 0)
    {
      if((rxbuf!= STX)&& (rxbuf!= ACK)&& (rxbuf!= NAK))
      {
        memset(rbuf3, 0, sizeof(rbuf3));

        return;
      }
    }

    if(etx_flag3)
    {
      etx_flag3=RESET;

      if(bccchk3!= bcc_check_buffer3) _bit_txnak3=SET;
      else _bit_txnak3=RESET;

      _chk_cmd3();
    }
    else
    {
      if((rxbuf== NAK)&& (rxlen3== 0))
      {
        if(POWER_CMD3) _bit_rxnak3=SET;
      }
      else
      {
        if(rxbuf== STX) rxlen3=0;

        if(rxlen3== 0) bccchk3=rxbuf;
        else bccchk3^=rxbuf;

        if(rxbuf== ETX) etx_flag3=SET;

        if(dle_flag3)
        {
          dle_flag3=RESET;
          rbuf3[rxlen3++]=rxbuf- 0x40;
        }
        else
        {
          if(rxbuf== DLE) dle_flag3=SET;
          else rbuf3[rxlen3++]=rxbuf;
        }
      }
    }
  }
}

void _chk_cmd3(void)
{
  if(_bit_txnak3== RESET)
  {
//    if(rbuf2[1]== 'p')
//    {
    switch(rbuf3[3])
    {
      case 'V':
        POWER_CMD3=p_Version;
        break;

      case 'E':
        POWER_CMD3=p_Error;
        break;

      case 'S':
        POWER_CMD3=p_Status;
        break;

      default:
        break;
    }
//    _uart.Fail_PSC1=RESET;
//    }
//    else _uart.PSC1=0x40; // command fail
  }

  if(_bit_txnak3== SET)
  {
    LL_USART_TransmitData8(USART3, NAK);
    _bit_txnak3=RESET;
  }
}

//------------------------------------------------------------------------------
// UART2 ROUTINE : PSC 2
//------------------------------------------------------------------------------
void _send_response5(void)
{
  tx5_done=0;

  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_6, txlen5);

  /* Enable DMA TX Interrupt */
  LL_USART_EnableDMAReq_TX(UART5);

  /* Enable DMA Channel Tx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_6);
}

/**
 * @brief  Function called at completion of last byte transmission
 * @param  None
 * @retval None
 */
//void USART3_CharTransmitComplete_Callback()
void DMA1_Channel6_TransmitCplt_Callback()
{
  /* Disable DMA Channel Tx */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);

  /* Clear buffer at end of transfer : Tx sequence completed successfully */
  memset(tbuf5, 0, sizeof(tbuf5));
  memset(tempbuf5, 0, sizeof(tempbuf5));

  tx5_done=1;
}

void DLE_encryption5(uint8_t *s, uint32_t len)
{
  uint8_t j=0, i=0;

  while(len--)
  {
    if((s[i]== STX)|| (s[i]== ETX)|| (s[i]== DLE))
    {
      if(i> 1)
      {
        tbuf5[j]=DLE;
        j++;
        tbuf5[j]=s[i]+ 0x40;
      }
      else tbuf5[j]=s[i];
    }
    else tbuf5[j]=s[i];
    i++;
    j++;
  }

  txlen5=j;
}

void _send_cmd5(uint8_t name)
{
  uint8_t TL=0;

  buflen5=0;
  tempbuf5[buflen5++]=STX;
  tempbuf5[buflen5++]='p';
  tempbuf5[buflen5++]=0x01;
  tempbuf5[buflen5++]=name;

  if(tx5_done)
  {
    DLE_encryption5(tempbuf5, buflen5);
    tbuf5[txlen5++]=ETX;
    TL=txlen5;
    tbuf5[txlen5++]=BCC_chk(tbuf5, TL);
    tx_rp5=0;
    _send_response5();
  }
  else DBG_PRINT("tx5 sending...1\r\n");
}

void _send_1byte5(uint8_t name, uint8_t _byte)
{
  uint8_t TL=0;

  buflen5=0;
  tempbuf5[buflen5++]=STX;
  tempbuf5[buflen5++]='p';
  tempbuf5[buflen5++]=0x02;
  tempbuf5[buflen5++]=name;
  tempbuf5[buflen5++]=_byte;

  if(tx5_done)
  {
    DLE_encryption5(tempbuf5, buflen5);
    tbuf5[txlen5++]=ETX;
    TL=txlen5;
    tbuf5[txlen5++]=BCC_chk(tbuf5, TL);
    tx_rp5=0;
    _send_response5();
  }
  else DBG_PRINT("tx5 sending...2\r\n");
}

void DMA1_Channel5_ReceiveCplt_Callback(void)
{
  uint8_t rxbuf=DMA_UART5_buf[0];
  static uint8_t dle_flag5=0, etx_flag5=0;

  bcc_check_buffer5=rxbuf;

  if(rxlen5< rx_buff_size5)
  {
    if(rxlen5== 0)
    {
      if((rxbuf!= STX)&& (rxbuf!= ACK)&& (rxbuf!= NAK))
      {
        memset(rbuf5, 0, rx_buff_size5);

        return;
      }
    }

    if(etx_flag5)
    {
      etx_flag5=RESET;

      if(bccchk5!= bcc_check_buffer5) _bit_txnak5=SET;
      else _bit_txnak5=RESET;

      _chk_cmd5();
    }
    else
    {
      if((rxbuf== NAK)&& (rxlen5== 0))
      {
        if(POWER_CMD5) _bit_rxnak5=SET;
      }
      else
      {
        if(rxbuf== STX) rxlen5=0;

        if(rxlen5== 0) bccchk5=rxbuf;
        else bccchk5^=rxbuf;

        if(rxbuf== ETX) etx_flag5=SET;

        if(dle_flag5)
        {
          dle_flag5=RESET;
          rbuf5[rxlen5++]=rxbuf- 0x40;
        }
        else
        {
          if(rxbuf== DLE) dle_flag5=SET;
          else rbuf5[rxlen5++]=rxbuf;
        }
      }
    }
  }
}

void _chk_cmd5(void)
{
  if(_bit_txnak5== RESET)
  {
//    if(rbuf3[1]== 'p')
//    {
    switch(rbuf5[3])
    {
      case 'V':
        POWER_CMD5=p_Version;
        break;

      case 'E':
        POWER_CMD5=p_Error;
        break;

      case 'S':
        POWER_CMD5=p_Status;
        break;

      default:
        break;
    }
//    _uart.Fail_PSC2=RESET;
//    }
//    else _uart.PSC2=0x40; // command fail
  }

  if(_bit_txnak5== SET)
  {
    LL_USART_TransmitData8(UART5, NAK);
    _bit_txnak5=RESET;
  }
}

/* End of Code */
