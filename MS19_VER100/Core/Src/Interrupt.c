//-------------------------------------------------------------------------
// Project    : MS19 Project (MainSystem Board)
//-------------------------------------------------------------------------
// MCU        : STM32G474QETx
// CLOCK      : 170MHz
// Date       : 2020.02.21
// Comfiler   : gcc (TBD)
// File Name  : Interrupt.c
// made by    : dohga
//-------------------------------------------------------------------------

#include "main.h"

uint32_t fanCountA=0, fanCountB=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance== TIM6)  // 200us
  {
    if(_master_flag== CHANNEL_L)
    {
      if(phase_2A_flag)
      {
        phase_2A_count++;
        if((pmr.mode== GRIP)|| (pmr.mode== TAP))
        {
          if((phase_2A_count== (pmr.s1* 500))&& (pmr.s2!= 0))
          {
            if(pmr.freq2!= pmr.freq1)
            {
              Q_change_A(pmr.freq2);
              changedFreqA=2;
            }
          }
          else if((phase_2A_count== ((pmr.s1+ pmr.s2)* 500))&& (pmr.s3!= 0))
          {
            if(pmr.freq3!= pmr.freq2)
            {
              Q_change_A(pmr.freq3);
              changedFreqA=3;
            }
          }
          else if((phase_2A_count== ((pmr.s1+ pmr.s2+ pmr.s3)* 500))&& (pmr.s4!= 0))
          {
            if(pmr.freq4!= pmr.freq3)
            {
              Q_change_A(pmr.freq4);
              changedFreqA=4;
            }
          }
          else if((phase_2A_count== ((pmr.s1+ pmr.s2+ pmr.s3+ pmr.s4)* 500))&& (pmr.s5!= 0))
          {
            if(pmr.freq5!= pmr.freq4)
            {
              Q_change_A(pmr.freq5);
              changedFreqA=5;
            }
          }
        }

        if(phase_2A_count>= (pmr.sTime* 500))
        {
          if(pmr.freq0!= 0)
          {
            phase_2A_count=0;
            phase_2A_flag=0;
            phase_3A_count=0;
            phase_3A_flag=1;

            if((pmr.mode== GRIP)|| (pmr.mode== TAP))
            {
              if(changedFreqA== 1)
              {
                if(pmr.freq0!= pmr.freq1) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
              else if(changedFreqA== 2)
              {
                if(pmr.freq0!= pmr.freq2) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
              else if(changedFreqA== 3)
              {
                if(pmr.freq0!= pmr.freq3) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
              else if(changedFreqA== 4)
              {
                if(pmr.freq0!= pmr.freq4) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
              else if(changedFreqA== 5)
              {
                if(pmr.freq0!= pmr.freq5) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
            }
          }
          else
          {
            // freq. 0 stop
          }
        }
      }

      if(phase_4A_flag)
      {
        phase_4A_count++;
        if(phase_4A_count>= (pmr.oTime* 500))
        {
          if(pmr.freq0!= 0)
          {
            if(reaction_A_flag)
            {
              reaction_A_flag=0;
              pmr.intenA=pmr.intenA_new;

              if(pmr.intenA> CONSTANT_DA) chA_uStep=(pmr.intenA- CONSTANT_DA)/ uStep;
              else chA_uStep=0;

              if(pmr.intenA> CONSTANT_DA) chA_dStep=(pmr.intenA- CONSTANT_DA)/ dStep;
              else chA_dStep=0;
            }
            phase_4A_count=0;
            phase_4A_flag=0;
            phase_1A_count=0;
            phase_1A_flag=1;
            Ready_A_Start();

            if(_next_step_)
            {
              _next_step_=0;
              Ready_B_Start();
            }
          }
        }
      }

      if(phase_2B_flag)
      {
        phase_2B_count++;
        if((pmr.mode== GRIP)|| (pmr.mode== TAP))
        {
          if((phase_2B_count== (pmr.s1* 500))&& (pmr.s2!= 0))
          {
            if(pmr.freq2!= pmr.freq1)
            {
              Q_change_B(pmr.freq2);
              changedFreqB=2;
            }
          }
          else if((phase_2B_count== ((pmr.s1+ pmr.s2)* 500))&& (pmr.s3!= 0))
          {
            if(pmr.freq3!= pmr.freq2)
            {
              Q_change_B(pmr.freq3);
              changedFreqB=3;
            }
          }
          else if((phase_2B_count== ((pmr.s1+ pmr.s2+ pmr.s3)* 500))&& (pmr.s4!= 0))
          {
            if(pmr.freq4!= pmr.freq3)
            {
              Q_change_B(pmr.freq4);
              changedFreqB=4;
            }
          }
          else if((phase_2B_count== ((pmr.s1+ pmr.s2+ pmr.s3+ pmr.s4)* 500))&& (pmr.s5!= 0))
          {
            if(pmr.freq5!= pmr.freq4)
            {
              Q_change_B(pmr.freq5);
              changedFreqB=5;
            }
          }
        }

        if(phase_2B_count>= (pmr.sTime* 500))
        {
          if(pmr.freq0!= 0)
          {
            phase_2B_count=0;
            phase_2B_flag=0;
            phase_3B_count=0;
            phase_3B_flag=1;

            if((pmr.mode== GRIP)|| (pmr.mode== TAP))
            {
              if(changedFreqB== 1)
              {
                if(pmr.freq0!= pmr.freq1) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
              else if(changedFreqB== 2)
              {
                if(pmr.freq0!= pmr.freq2) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
              else if(changedFreqB== 3)
              {
                if(pmr.freq0!= pmr.freq3) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
              else if(changedFreqB== 4)
              {
                if(pmr.freq0!= pmr.freq4) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
              else if(changedFreqB== 5)
              {
                if(pmr.freq0!= pmr.freq5) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
            }
          }
          else
          {
            // freq. 0 stop
          }
        }
      }

      if(phase_4B_flag)
      {
        phase_4B_count++;
        if(phase_4B_count>= (pmr.oTime* 500))
        {
          if(pmr.freq0!= 0)
          {
            if(reaction_B_flag)
            {
              reaction_B_flag=0;
              pmr.intenB=pmr.intenB_new;

              if(pmr.intenB> CONSTANT_DA) chB_uStep=(pmr.intenB- CONSTANT_DA)/ uStep;
              else chB_uStep=0;

              if(pmr.intenB> CONSTANT_DA) chB_dStep=(pmr.intenB- CONSTANT_DA)/ dStep;
              else chB_dStep=0;
            }
            phase_4B_count=0;
            phase_4B_flag=0;
            phase_1B_count=0;
            phase_1B_flag=1;
            Ready_B_Start();

            if(_next_step_)
            {
              _next_step_=0;
              Ready_A_Start();
            }
          }
        }
      }
    }
    else  // when the CHANNEL_R is master
    {
      if(phase_2B_flag)
      {
        phase_2B_count++;
        if((pmr.mode== GRIP)|| (pmr.mode== TAP))
        {
          if((phase_2B_count== (pmr.s1* 500))&& (pmr.s2!= 0))
          {
            if(pmr.freq2!= pmr.freq1)
            {
              Q_change_B(pmr.freq2);
              changedFreqB=2;
            }
          }
          else if((phase_2B_count== ((pmr.s1+ pmr.s2)* 500))&& (pmr.s3!= 0))
          {
            if(pmr.freq3!= pmr.freq2)
            {
              Q_change_B(pmr.freq3);
              changedFreqB=3;
            }
          }
          else if((phase_2B_count== ((pmr.s1+ pmr.s2+ pmr.s3)* 500))&& (pmr.s4!= 0))
          {
            if(pmr.freq4!= pmr.freq3)
            {
              Q_change_B(pmr.freq4);
              changedFreqB=4;
            }
          }
          else if((phase_2B_count== ((pmr.s1+ pmr.s2+ pmr.s3+ pmr.s4)* 500))&& (pmr.s5!= 0))
          {
            if(pmr.freq5!= pmr.freq4)
            {
              Q_change_B(pmr.freq5);
              changedFreqB=5;
            }
          }
        }

        if(phase_2B_count>= (pmr.sTime* 500))
        {
          if(pmr.freq0!= 0)
          {
            phase_2B_count=0;
            phase_2B_flag=0;
            phase_3B_count=0;
            phase_3B_flag=1;

            if((pmr.mode== GRIP)|| (pmr.mode== TAP))
            {
              if(changedFreqB== 1)
              {
                if(pmr.freq0!= pmr.freq1) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
              else if(changedFreqB== 2)
              {
                if(pmr.freq0!= pmr.freq2) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
              else if(changedFreqB== 3)
              {
                if(pmr.freq0!= pmr.freq3) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
              else if(changedFreqB== 4)
              {
                if(pmr.freq0!= pmr.freq4) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
              else if(changedFreqB== 5)
              {
                if(pmr.freq0!= pmr.freq5) Q_change_B(pmr.freq0);
                changedFreqB=0;
              }
            }
          }
          else
          {
            // freq. 0 stop
          }
        }
      }

      if(phase_4B_flag)
      {
        phase_4B_count++;
        if(phase_4B_count>= (pmr.oTime* 500))
        {
          if(pmr.freq0!= 0)
          {
            if(reaction_B_flag)
            {
              reaction_B_flag=0;
              pmr.intenB=pmr.intenB_new;

              if(pmr.intenB> CONSTANT_DA) chB_uStep=(pmr.intenB- CONSTANT_DA)/ uStep;
              else chB_uStep=0;

              if(pmr.intenB> CONSTANT_DA) chB_dStep=(pmr.intenB- CONSTANT_DA)/ dStep;
              else chB_dStep=0;
            }
            phase_4B_count=0;
            phase_4B_flag=0;
            phase_1B_count=0;
            phase_1B_flag=1;
            Ready_B_Start();

            if(_next_step_)
            {
              _next_step_=0;
              Ready_A_Start();
            }
          }
        }
      }

      if(phase_2A_flag)
      {
        phase_2A_count++;
        if((pmr.mode== GRIP)|| (pmr.mode== TAP))
        {
          if((phase_2A_count== (pmr.s1* 500))&& (pmr.s2!= 0))
          {
            if(pmr.freq2!= pmr.freq1)
            {
              Q_change_A(pmr.freq2);
              changedFreqA=2;
            }
          }
          else if((phase_2A_count== ((pmr.s1+ pmr.s2)* 500))&& (pmr.s3!= 0))
          {
            if(pmr.freq3!= pmr.freq2)
            {
              Q_change_A(pmr.freq3);
              changedFreqA=3;
            }
          }
          else if((phase_2A_count== ((pmr.s1+ pmr.s2+ pmr.s3)* 500))&& (pmr.s4!= 0))
          {
            if(pmr.freq4!= pmr.freq3)
            {
              Q_change_A(pmr.freq4);
              changedFreqA=4;
            }
          }
          else if((phase_2A_count== ((pmr.s1+ pmr.s2+ pmr.s3+ pmr.s4)* 500))&& (pmr.s5!= 0))
          {
            if(pmr.freq5!= pmr.freq4)
            {
              Q_change_A(pmr.freq5);
              changedFreqA=5;
            }
          }
        }

        if(phase_2A_count>= (pmr.sTime* 500))
        {
          if(pmr.freq0!= 0)
          {
            phase_2A_count=0;
            phase_2A_flag=0;
            phase_3A_count=0;
            phase_3A_flag=1;

            if((pmr.mode== GRIP)|| (pmr.mode== TAP))
            {
              if(changedFreqA== 1)
              {
                if(pmr.freq0!= pmr.freq1) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
              else if(changedFreqA== 2)
              {
                if(pmr.freq0!= pmr.freq2) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
              else if(changedFreqA== 3)
              {
                if(pmr.freq0!= pmr.freq3) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
              else if(changedFreqA== 4)
              {
                if(pmr.freq0!= pmr.freq4) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
              else if(changedFreqA== 5)
              {
                if(pmr.freq0!= pmr.freq5) Q_change_A(pmr.freq0);
                changedFreqA=0;
              }
            }
          }
          else
          {
            // freq. 0 stop
          }
        }
      }

      if(phase_4A_flag)
      {
        phase_4A_count++;
        if(phase_4A_count>= (pmr.oTime* 500))
        {
          if(pmr.freq0!= 0)
          {
            if(reaction_A_flag)
            {
              reaction_A_flag=0;
              pmr.intenA=pmr.intenA_new;

              if(pmr.intenA> CONSTANT_DA) chA_uStep=(pmr.intenA- CONSTANT_DA)/ uStep;
              else chA_uStep=0;

              if(pmr.intenA> CONSTANT_DA) chA_dStep=(pmr.intenA- CONSTANT_DA)/ dStep;
              else chA_dStep=0;
            }
            phase_4A_count=0;
            phase_4A_flag=0;
            phase_1A_count=0;
            phase_1A_flag=1;
            Ready_A_Start();

            if(_next_step_)
            {
              _next_step_=0;
              Ready_B_Start();
            }
          }
        }
      }
    }
  }
}

// period: 1ms
void HAL_IncTick(void)
{
  uwTick+=uwTickFreq;
  if(uwTick>= 15000)
  {
    uwTick=0;
    check15sec=1;
  }

  /* for test */
  hb_cnt++;
  if(hb_cnt>= 499)
  {
    hb_cnt=0;
    if(hb_flag)
    {
      hb_flag=0;
      LD2_OFF;
    }
    else
    {
      hb_flag=1;
      LD2_ON;
    }
  }

#if 0
  if(AP_A_Fan_Active)
  {
    fanCountA++;
    if(fanCountA>= 10000)
    {
      fanCountA=0;
      if(SYSTEM_A_MODE!= READY_MODE) AP_A_FAN_START;
      else
      {
        if(LL_GPIO_IsOutputPinSet(AP_A_FAN_ctl_GPIO_Port, AP_A_FAN_ctl_Pin)) AP_A_FAN_STOP;
        else AP_A_FAN_START;
      }
    }
  }
  else fanCountA=0;
#endif
#if 0
  if(AP_B_Fan_Active)
  {
    fanCountB++;
    if(fanCountB>= 10000)
    {
      fanCountB=0;
      if(SYSTEM_B_MODE!= READY_MODE) AP_B_FAN_START;
      else
      {
        if(LL_GPIO_IsOutputPinSet(AP_B_FAN_ctl_GPIO_Port, AP_B_FAN_ctl_Pin)) AP_B_FAN_STOP;
        else AP_B_FAN_START;
      }
    }
  }
  else fanCountB=0;
#endif

  // when the power-switch pressed
  if(waitingACK&& !waitingACK_done)
  {
    waitingACK_count++;
    if(waitingACK_count> ChkTime_150sec)
    {
      waitingACK_timeover=1;
      waitingACK_count=ChkTime_150sec;
    }
  }
  else waitingACK_count=0;

  if(power_on_flag)
  {
    power_on_count++;
    if(power_on_count>= 400)
    {
      power_on_count=0;
      if(LL_GPIO_IsInputPinSet(Pwr_Btn_LED_GPIO_Port, Pwr_Btn_LED_Pin)) PUSH_LED_OFF;
      else PUSH_LED_ON;
    }
  }

  if(power_off_flag)
  {
    power_off_count++;
    if(power_off_count>= 100)
    {
      power_off_count=0;
      if(LL_GPIO_IsInputPinSet(Pwr_Btn_LED_GPIO_Port, Pwr_Btn_LED_Pin)) PUSH_LED_OFF;
      else PUSH_LED_ON;
    }
  }

  // when the jog push-switch pressed
  encoder_count++;
  if(encoder_count>= (2- 1))
  {
    encoder_count=0;
    encoder_flag=1;
  }

  /* I/Os check block start*/
  if(veryfirst_boot)
  {
    if(LL_GPIO_IsInputPinSet(KEY_CHECK_GPIO_Port, KEY_CHECK_Pin))
    {
      veryfirst_boot=0;
      _io_CHK._bPwrButton_CNT=RESET;
    }
  }
  else
  {
    if(!LL_GPIO_IsInputPinSet(KEY_CHECK_GPIO_Port, KEY_CHECK_Pin))
    {
      _io_CHK._bPwrButton_CNT++;
      if(_io_CHK._bPwrButton_CNT> ChkTime_400ms) _io_CHK._bPwrButton_CNT=ChkTime_400ms;
    }
    else _io_CHK._bPwrButton_CNT=RESET;
  }

  if(LL_GPIO_IsInputPinSet(Jog_PushButton_GPIO_Port, Jog_PushButton_Pin))
  {
    _io_CHK._bJogPushSW_CNT++;
    if(_io_CHK._bJogPushSW_CNT> ChkTime_10ms) _io_CHK._bJogPushSW_CNT=ChkTime_10ms;
  }
  else
  {
    Encoder_SW_flag=0;
    _io_CHK._bJogPushSW_CNT=RESET;
  }

  if(!LL_GPIO_IsInputPinSet(AP_A_CONNECT_GPIO_Port, AP_A_CONNECT_Pin))
  {
    _io_CHK.bAP_A_conn_CNT++;
    if(_io_CHK.bAP_A_conn_CNT> ChkTime_200ms) _io_CHK.bAP_A_conn_CNT=ChkTime_200ms;
  }
  else _io_CHK.bAP_A_conn_CNT=RESET;

  if(!LL_GPIO_IsInputPinSet(AP_B_CONNECT_GPIO_Port, AP_B_CONNECT_Pin))
  {
    _io_CHK.bAP_B_conn_CNT++;
    if(_io_CHK.bAP_B_conn_CNT> ChkTime_200ms) _io_CHK.bAP_B_conn_CNT=ChkTime_200ms;
  }
  else _io_CHK.bAP_B_conn_CNT=RESET;

//  if(!LL_GPIO_IsInputPinSet(Patient_interlock_GPIO_Port, Patient_interlock_Pin))
//  {
//    _io_CHK._bPatientSW_CNT++;
//    if(_io_CHK._bPatientSW_CNT> ChkTime_500ms) _io_CHK._bPatientSW_CNT=ChkTime_500ms;
//  }
//  else _io_CHK._bPatientSW_CNT=RESET;

  if(!LL_GPIO_IsInputPinSet(Door_interlock_R_GPIO_Port, Door_interlock_R_Pin))
  {
    _io_CHK._bDoorOpenR_CNT++;
    if(_io_CHK._bDoorOpenR_CNT> ChkTime_200ms) _io_CHK._bDoorOpenR_CNT=ChkTime_200ms;
  }
  else _io_CHK._bDoorOpenR_CNT=RESET;

  if(!LL_GPIO_IsInputPinSet(Door_interlock_L_GPIO_Port, Door_interlock_L_Pin))
  {
    _io_CHK._bDoorOpenL_CNT++;
    if(_io_CHK._bDoorOpenL_CNT> ChkTime_200ms) _io_CHK._bDoorOpenL_CNT=ChkTime_200ms;
  }
  else _io_CHK._bDoorOpenL_CNT=RESET;

  if(LL_GPIO_IsInputPinSet(AC_STATE_CHECK_GPIO_Port, AC_STATE_CHECK_Pin))
  {
    _io_CHK._bContactor_CNT++;
    if(_io_CHK._bContactor_CNT> ChkTime_1sec) _io_CHK._bContactor_CNT=ChkTime_1sec;
  }
  else
  {
    _io_CHK._bContactor_CNT=RESET;
    if(EF2.bError27== SET) EF2.bError27=RESET;
  }

  /* I/Os check block end*/

  // Status & Error Check Delay Time
  _io_CHK._bCheckDelay++;
  if(_io_CHK._bCheckDelay> ChkTime_500ms) _io_CHK._bCheckDelay=ChkTime_500ms;
}

/* End of Code */
