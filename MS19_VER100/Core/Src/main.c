/* USER CODE BEGIN Header */
//-------------------------------------------------------------------------
// Project    : MS19 Project (MainSystem Board)
//-------------------------------------------------------------------------
// MCU        : STM32G474QETx
// CLOCK      : 170MHz
// Date       : 2020.02.21
// Comfiler   : gcc (TBD)
// File Name  : main.c
// made by    : dohga
//-------------------------------------------------------------------------
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifdef USE_ADC_CONV
/* Timeout values for ADC operations. */
/* (calibration, enable settling time, disable settling time, ...)          */
/* Values defined to be higher than worst cases: low clock frequency,       */
/* maximum prescalers.                                                      */
/* Example of profile very low frequency : ADC clock frequency 0.14MHz      */
/* prescaler 256, sampling time 247.5 ADC clock cycles, resolution 12 bits. */
/*  - ADC calibration time: On STM32G4 ADC, maximum delay is 112/fADC,      */
/*    resulting in a maximum delay of 800us                                 */
/*    (refer to device datasheet, parameter "tCAL")                         */
/*  - ADC enable time: maximum delay is 1 conversion cycle.                 */
/*    (refer to device datasheet, parameter "tSTAB")                        */
/*  - ADC disable time: maximum delay should be a few ADC clock cycles      */
/*  - ADC stop conversion time: maximum delay should be a few ADC clock     */
/*    cycles                                                                */
/*  - ADC conversion time: with this hypothesis of clock settings, maximum  */
/*    delay will be 476ms.                                                  */
/*    (refer to device reference manual, section "Timing")                  */
/* Unit: ms                                                                 */
#define ADC_CALIBRATION_TIMEOUT_MS       (   1U)
#define ADC_ENABLE_TIMEOUT_MS            (   1U)
//#define ADC_DISABLE_TIMEOUT_MS           (   1U)
//#define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1U)
//#define ADC_CONVERSION_TIMEOUT_MS        ( 500U)

/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)

/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300UL)

/* Definitions of data related to this example */
/* Full-scale digital value with a resolution of 12 bits (voltage range     */
/* determined by analog voltage references Vref+ and Vref-,                 */
/* refer to reference manual).                                              */
//#define DIGITAL_SCALE_12BITS             (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B))
/* Init variable out of ADC expected conversion data range */
//#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)
#endif

//#define USE_LPUART

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc5;

I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

/* USER CODE BEGIN PV */

uint8_t bBuzzerRefresh=0;

#ifdef USE_LPUART
uint8_t LP_RX_Buffer[1]={0, };
#else
uint8_t RX4_Buffer[1]={0, };
#endif
uint8_t AP_A_LED=0, AP_B_LED=0;
uint8_t first_bootA=0, first_bootB=0;

#ifdef USE_ADC_CONV
__IO uint16_t DMA_ADC2_Buf[ADC_COUNT* 2]={0, };

uint16_t adc2ValA[ADC_COUNT]={0, };
uint16_t adc2ValB[ADC_COUNT]={0, };

uint16_t hallA_low[HALL_COUNT]={0, };
uint16_t hallB_low[HALL_COUNT]={0, };
uint16_t hallA_cnt=0, hallB_cnt=0;
__IO uint8_t sendHallA=0, sendHallB=0, clearHallA=0;

uint16_t adc1Val[2]={0, };
uint8_t adc1Ch=0, adc1Done=0;

uint16_t adc5Val[2]={0, };
uint8_t adc5Ch=0, adc5Done=0;

uint8_t underTempA=0, underTempB=0, overTempA=0, overTempB=0;
uint16_t uLimit=0;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

void Activate_ADC(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t byte2int(uint8_t high, uint8_t low)
{
  uint32_t intbuffer=0;

  intbuffer=high;
  intbuffer<<=8;
  intbuffer+=low;

  return intbuffer;
}

int static compare(const void *first, const void *second)
{
  if(*(uint16_t*)first> *(uint16_t*)second)
  {
    return 1;
  }
  else if(*(uint16_t*)first< *(uint16_t*)second)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

void value_init()
{
  Ready_A_Stop();
  POWER_DA_CONTROL_A(RESET);
  Ready_B_Stop();
  POWER_DA_CONTROL_B(RESET);

  memset(rbuf1, 0, rx_buff_size1);
  memset(rbuf3, 0, rx_buff_size3);
  memset(rbuf5, 0, rx_buff_size5);

  PSC_Version1=0;
  PSC_Version2=0;

  _bit_txnak1=0;
  _bit_rxnak1=0;
  buflen1=0;
  bccchk1=0;
  tx_rp1=0;
  bcc_check_buffer1=0;

  _bit_txnak3=0;
  _bit_rxnak3=0;
  buflen3=0;
  bccchk3=0;
  tx_rp3=0;
  bcc_check_buffer3=0;

  _bit_txnak5=0;
  _bit_rxnak5=0;
  buflen5=0;
  bccchk5=0;
  tx_rp5=0;
  bcc_check_buffer5=0;

  encoderValue=0;
  power_off_flag=0;
  power_off_count=0;

  waitingACK=0;
  waitingACK_done=0;
  waitingACK_timeover=0;
  waitingACK_count=0;

  // System Mode
  CH_LEFT=STOP_MODE;
  old_CH_L=CH_LEFT;
  CH_RIGHT=STOP_MODE;
  old_CH_R=CH_RIGHT;

  SF1.STATUS1B=0;
  oldSTATUS1B=255;
  SF2.STATUS2B=0;
  oldSTATUS2B=255;
  SF3.STATUS3B=0;
  oldSTATUS3B=255;
  SF4.STATUS4B=0;
  oldSTATUS4B=255;
  sendStatus=0;

  EF1.ERROR1B=0;
  oldERROR1B=0;
  EF2.ERROR2B=0;
  oldERROR2B=0;
  EF3.ERROR3B=0;
  oldERROR3B=0;
  EF4.ERROR4B=0;
  oldERROR4B=0;
  sendError=0;

  // Serial Status Buffer Iint
  _bit_txnak1=RESET;
  _bit_txnak3=RESET;
  _bit_txnak5=RESET;

  // PSC Status Buffer Clear
  bBuzzerRefresh=SET;

  AC_VoltBuffer1=0; //220;
  AC_VoltBuffer2=0; //220;
  PSC_VerCheck1=0;
  PSC_VerCheck2=0;

  // default: enable both channels
  _io_CHK.bAP_A_conn_CNT=RESET;
  _io_CHK.bAP_B_conn_CNT=RESET;

  operation_flag=0;

  AP_A_Fan_Active=0;
  AP_A_Temp=1;
  AP_B_Fan_Active=0;
  AP_B_Temp=1;

  veryfirst_boot=1;
  GUI_Boot_OK=0;

  _twist_once_=0;
  _next_step_=0;
  _master_flag=0;

  reaction_A_flag=0;
  reaction_B_flag=0;

  pending_flag=0;

  pmr.mode=255;
  pmr.freq0=30;

  pwr_btn_pressed=0;
  ver_count=0;

  for(int i=0; i< HALL_COUNT; i++)
  {
    hallA_low[i]=VDDA_APPLI;
    hallB_low[i]=VDDA_APPLI;
  }

  adc5Val[0]=VDDA_APPLI;

  intenA_Zero=0;
  intenB_Zero=0;

  pmr.linked=0;
  dischargeA=0;
  dischargeB=0;

  dStep5=0;
  chA_dStep5=0;
  chB_dStep5=0;
  phase_5A_flag=0;
  phase_5B_flag=0;
  phase_5A_count=0;
  phase_5B_count=0;

  intenA_reply_cnt=0;
  intenB_reply_cnt=0;

  chA_DA_zero=0;
  chB_DA_zero=0;
}

void AP_A_FAN_ON()
{
  AP_A_Fan_Active=1;

  AP_A_FAN_START;
}

void AP_A_FAN_OFF()
{
  AP_A_FAN_STOP;

  AP_A_Fan_Active=0;
  AP_A_Temp=1;
  underTempA=0;
}

void AP_B_FAN_ON()
{
  AP_B_Fan_Active=1;

  AP_B_FAN_START;
}

void AP_B_FAN_OFF()
{
  AP_B_FAN_STOP;

  AP_B_Fan_Active=0;
  AP_B_Temp=1;
  underTempB=0;
}

void RunToStopA()
{
  /* Stop channel 1 (Q-pulse-A) */
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  TIM1->CNT=0;
  Ready_A_Stop();
  POWER_DA_CONTROL_A(0);

  phase_1A_flag=0;
  phase_1A_count=0;
  phase_2A_flag=0;
  phase_2A_count=0;
  phase_3A_flag=0;
  phase_3A_count=0;
  phase_4A_flag=0;
  phase_4A_count=0;
  chA_uStep=0;
  chA_dStep=0;
  chA_DA=0;
  once_A_flag=0;

  //if(AP_A_Fan_Active&& AP_A_Temp) AP_A_FAN_OFF();

  reaction_A_flag=0;
  sendHallA=0;
  overTempA=0;

  dischargeA=0;
}

void RunToStopB()
{
  /* Stop channel 1 (Q-pulse-B) */
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
  TIM8->CNT=0;
  Ready_B_Stop();
  POWER_DA_CONTROL_B(0);

  phase_1B_flag=0;
  phase_1B_count=0;
  phase_2B_flag=0;
  phase_2B_count=0;
  phase_3B_flag=0;
  phase_3B_count=0;
  phase_4B_flag=0;
  phase_4B_count=0;
  chB_uStep=0;
  chB_dStep=0;
  chB_DA=0;
  once_B_flag=0;

  //if(AP_B_Fan_Active&& AP_B_Temp) AP_B_FAN_OFF();

  reaction_B_flag=0;
  sendHallB=0;
  overTempB=0;

  dischargeB=0;
}

void RunToStopVal()
{
  pmr.mode=255;
  _twist_once_=0;
  _next_step_=0;
  _master_flag=0;
  pending_flag=0;

  intenA_Zero=0;
  intenB_Zero=0;

  pmr.linked=0;

  chA_DA_zero=0;
  chB_DA_zero=0;
}

void RunToPause(uint8_t ch)
{
  if((ch== 1)|| (ch== 3))
  {
//    /* Stop channel 1 (Q-pulse-A) */
//    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//    TIM1->CNT=0;
    Ready_A_Stop();
    chA_DA=0;
    POWER_DA_CONTROL_A(chA_DA);

    phase_1A_flag=0;
    phase_1A_count=0;
    phase_2A_flag=0;
    phase_2A_count=0;
    phase_3A_flag=0;
    phase_3A_count=0;
    phase_4A_flag=0;
    phase_4A_count=0;
    chA_uStep=0;
    chA_dStep=0;
    once_A_flag=0;
    reaction_A_flag=0;
    sendHallA=0;
    overTempA=0;

    dischargeA=0;
  }

  if((ch== 2)|| (ch== 3))
  {
//    /* Stop channel 1 (Q-pulse-B) */
//    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
//    TIM8->CNT=0;
    Ready_B_Stop();
    chB_DA=0;
    POWER_DA_CONTROL_B(chB_DA);

    phase_1B_flag=0;
    phase_1B_count=0;
    phase_2B_flag=0;
    phase_2B_count=0;
    phase_3B_flag=0;
    phase_3B_count=0;
    phase_4B_flag=0;
    phase_4B_count=0;
    chB_uStep=0;
    chB_dStep=0;
    once_B_flag=0;
    reaction_B_flag=0;
    sendHallB=0;
    overTempB=0;

    dischargeB=0;
  }

  if((CH_LEFT== PAUSE_MODE)&& (CH_RIGHT== PAUSE_MODE)) RunToStopVal();
//  {
//    operation_flag=1;
////    _twist_once_=0;
//    _next_step_=0;
////    _master_flag=0;
//    pending_flag=0;
//  }
}

void RunToRest(uint8_t ch)
{
  if(ch== 1)
  {
    Ready_A_Stop();
    if(old_CH_L== PAUSE_MODE) chA_DA=0;
    else if(chA_DA_zero) chA_DA=0;
    else chA_DA=CONSTANT_DA;
    POWER_DA_CONTROL_A(chA_DA);

    phase_1A_flag=0;
    phase_1A_count=0;
    phase_2A_flag=0;
    phase_2A_count=0;
    phase_3A_flag=0;
    phase_3A_count=0;
    phase_4A_flag=0;
    phase_4A_count=0;
    chA_uStep=0;
    chA_dStep=0;
    once_A_flag=0;

    reaction_A_flag=0;

    sendHallA=0;
    overTempA=0;
  }

  if(ch== 2)
  {
    Ready_B_Stop();
    if(old_CH_R== PAUSE_MODE) chB_DA=0;
    else if(chB_DA_zero) chB_DA=0;
    else chB_DA=CONSTANT_DA;
    POWER_DA_CONTROL_B(chB_DA);

    phase_1B_flag=0;
    phase_1B_count=0;
    phase_2B_flag=0;
    phase_2B_count=0;
    phase_3B_flag=0;
    phase_3B_count=0;
    phase_4B_flag=0;
    phase_4B_count=0;
    chB_uStep=0;
    chB_dStep=0;
    once_B_flag=0;

    reaction_B_flag=0;

    sendHallB=0;
    overTempB=0;
  }

  pmr.mode=200;
  _twist_once_=0;
//  _next_step_=0;
//  _master_flag=0;
  pending_flag=0;
}

void System_Off()
{
//  // Discahrging
//  POWER_DA_CONTROL_A(0);
//  POWER_DA_CONTROL_B(0);
//  delay_ms(100);
//
//  Ready_A_Stop();
//  Ready_B_Stop();
//
//  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);

  if(waitingACK_timeover)
  {
    LOG_PRINT("FAIL\r\n");
    LOG_PRINT("GUI forced Power-OFF\r\n");
  }
  PC_SW_ON;
  delay_ms(500);
  PC_SW_OFF;

  LOG_PRINT("AC Contactor OFF\r\n");
  CONTACTOR_OFF;

  // GUI off time: about 10 sec
  LOG_PRINT("Wait 10s for the GUI to turn off...");
  delay_ms(5000);
  delay_ms(5000);
  LOG_PRINT("DONE\r\n");

  LOG_PRINT("Front LED OFF\r\n");
  FRONT_LED_OFF;

  LOG_PRINT("Power-Button LED OFF\r\n");
  PUSH_LED_OFF;

  BUZZER_ON;
  delay_ms(100);
  BUZZER_OFF;
  delay_ms(100);
  BUZZER_ON;
  delay_ms(100);
  BUZZER_OFF;

  LOG_PRINT("-- Shutdown --\r\n");

  __disable_irq();
  HAL_PWREx_DisableInternalWakeUpLine();
  HAL_PWREx_EnterSHUTDOWNMode();

  REMOTE_OFF;
}

void Pwr_Btn_Check()
{
  if(pwr_btn_pressed)
  {
    if(LL_GPIO_IsInputPinSet(KEY_CHECK_GPIO_Port, KEY_CHECK_Pin))
    {
      if(!waitingACK)
      {
        power_off_flag=1;
        power_off_count=0;
        LOG_PRINT("\nMain System Power-OFF seq.\r\n");
        LOG_PRINT("----------------------------\r\n");

        RunToStopA();
        RunToStopB();
        RunToStopVal();

        SEND_1BYTE(CMD_GUI_ONOFF, 1);
        waitingACK=1;
        LOG_PRINT("GUI Ack waiting...");
      }
    }
  }

  if(waitingACK)
  {
    if(waitingACK_done|| waitingACK_timeover) System_Off();
  }

  if(_io_CHK._bPwrButton_CNT>= ChkTime_400ms)
  {
    if(pwr_btn_pressed== 0)
    {
      BUZZER_ON;
      delay_ms(100);
      BUZZER_OFF;
    }
//    if(AP_A_Fan_Active|| AP_B_Fan_Active) SEND_1BYTE(CMD_TEMPER, 1);
//    else
    pwr_btn_pressed=1;
  }
}

void Status_Check()
{
  if(_io_CHK.bAP_A_conn_CNT>= ChkTime_200ms)
  {
    SF1.bStatus11=RESET;

    if(CH_LEFT== RUN_MODE)
    {
      LL_GPIO_TogglePin(AP_A_LED1_GPIO_Port, AP_A_LED1_Pin);
      LL_GPIO_TogglePin(AP_A_LED2_GPIO_Port, AP_A_LED2_Pin);
      LL_GPIO_TogglePin(AP_A_LED3_GPIO_Port, AP_A_LED3_Pin);
      AP_A_LED=2;
    }
    else if(AP_A_LED!= 1)
    {
      LL_GPIO_SetOutputPin(AP_A_LED1_GPIO_Port, AP_A_LED1_Pin);
      LL_GPIO_SetOutputPin(AP_A_LED2_GPIO_Port, AP_A_LED2_Pin);
      LL_GPIO_SetOutputPin(AP_A_LED3_GPIO_Port, AP_A_LED3_Pin);
      AP_A_LED=1;

      if(!first_bootA)
      {
        AP_A_FAN_ON();
        first_bootA=1;
      }
    }
  }
  else
  {
    SF1.bStatus11=SET;

    if(AP_A_LED!= 0)
    {
      LL_GPIO_ResetOutputPin(AP_A_LED1_GPIO_Port, AP_A_LED1_Pin);
      LL_GPIO_ResetOutputPin(AP_A_LED2_GPIO_Port, AP_A_LED2_Pin);
      LL_GPIO_ResetOutputPin(AP_A_LED3_GPIO_Port, AP_A_LED3_Pin);
      AP_A_LED=0;
    }
  }

  if(_io_CHK.bAP_B_conn_CNT>= ChkTime_200ms)
  {
    SF1.bStatus12=RESET;

    if(CH_RIGHT== RUN_MODE)
    {
      LL_GPIO_TogglePin(AP_B_LED1_GPIO_Port, AP_B_LED1_Pin);
      LL_GPIO_TogglePin(AP_B_LED2_GPIO_Port, AP_B_LED2_Pin);
      LL_GPIO_TogglePin(AP_B_LED3_GPIO_Port, AP_B_LED3_Pin);
      AP_B_LED=2;
    }
    else if(AP_B_LED!= 1)
    {
      LL_GPIO_SetOutputPin(AP_B_LED1_GPIO_Port, AP_B_LED1_Pin);
      LL_GPIO_SetOutputPin(AP_B_LED2_GPIO_Port, AP_B_LED2_Pin);
      LL_GPIO_SetOutputPin(AP_B_LED3_GPIO_Port, AP_B_LED3_Pin);
      AP_B_LED=1;

      if(!first_bootB)
      {
        AP_B_FAN_ON();
        first_bootB=1;
      }
    }
  }
  else
  {
    SF1.bStatus12=SET;

    if(AP_B_LED!= 0)
    {
      LL_GPIO_ResetOutputPin(AP_B_LED1_GPIO_Port, AP_B_LED1_Pin);
      LL_GPIO_ResetOutputPin(AP_B_LED2_GPIO_Port, AP_B_LED2_Pin);
      LL_GPIO_ResetOutputPin(AP_B_LED3_GPIO_Port, AP_B_LED3_Pin);
      AP_B_LED=0;
    }
  }

  if((oldSTATUS1B!= SF1.STATUS1B)|| (oldSTATUS2B!= SF2.STATUS2B)|| (oldSTATUS3B!= SF3.STATUS3B)|| (oldSTATUS4B!= SF4.STATUS4B))
  {
    sendStatus=1;
    if(GUI_Boot_OK)
    {
      SEND_STATUS();
      DBG_PRINT("Send Status1\r\n");
    }
    oldSTATUS1B=SF1.STATUS1B;
    oldSTATUS2B=SF2.STATUS2B;
    oldSTATUS3B=SF3.STATUS3B;
    oldSTATUS4B=SF4.STATUS4B;
  }
  else if(sendStatus&& GUI_Boot_OK)
  {
    SEND_STATUS();
    if(use_debug) DBG_PRINT("Send Status2\r\n");
  }
}

// Always Check
void Error_Check()
{
  if(!((_io_CHK._bDoorOpenR_CNT>= ChkTime_200ms)&& (_io_CHK._bDoorOpenL_CNT>= ChkTime_200ms))) EF2.bError26=SET;

  if(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_4))
  {
    if(_io_CHK._bContactor_CNT>= ChkTime_1sec) EF2.bError27=SET;
  }

  if((CH_LEFT== RUN_MODE)&& SF1.bStatus11) EF3.bError31=SET;
  if((CH_RIGHT== RUN_MODE)&& SF1.bStatus12) EF3.bError32=SET;

  if((oldERROR1B!= EF1.ERROR1B)|| (oldERROR2B!= EF2.ERROR2B)|| (oldERROR3B!= EF3.ERROR3B)|| (oldERROR4B!= EF4.ERROR4B))
  {
    // stop All
    RunToStopA();
    RunToStopB();
    RunToStopVal();

    CH_LEFT=ERROR_MODE;
    CH_RIGHT=ERROR_MODE;

    sendError=1;
    if(GUI_Boot_OK)
    {
      SEND_ERROR();
      DBG_PRINT("Send Error1\r\n");
    }
    oldERROR1B=EF1.ERROR1B;
    oldERROR2B=EF2.ERROR2B;
    oldERROR3B=EF3.ERROR3B;
    oldERROR4B=EF4.ERROR4B;
  }
  else if(sendError&& GUI_Boot_OK)
  {
    SEND_ERROR();
    if(use_debug) DBG_PRINT("Send Error2\r\n");
  }
}

void Error_Sound_cnt()
{
#if 0
  if(EF1.bERROR_SysFan1)
  {
    _io_CHK._BuzzerCNT=1;
    //return;
  }
  if(EF1.bERROR_SysFan2)
  {
    _io_CHK._BuzzerCNT=2;
    //return;
  }
  if(EF1.bERROR_AP_A_Fan1)
  {
    _io_CHK._BuzzerCNT=3;
    //return;
  }
  if(EF1.bERROR_AP_A_Fan2)
  {
    _io_CHK._BuzzerCNT=4;
    //return;
  }
  if(EF1.bERROR_AP_B_Fan1)
  {
    _io_CHK._BuzzerCNT=5;
    //return;
  }
  if(EF1.bERROR_AP_B_Fan2)
  {
    _io_CHK._BuzzerCNT=6;
    //return;
  }
  if(EF1.bERROR_Contactor)
  {
    _io_CHK._BuzzerCNT=7;
    //return;
  }
#else
  _io_CHK._BuzzerCNT=0;
#endif
}

//void Error_Reset(void)
//{
//   _io_CHK._bCheckDelay=RESET;
//   EF1.byERROR_FLAG1=RESET;
////   _uart.Error2=RESET;
////   _uart.HDstatus=RESET;
//   _uart.PSC=RESET;
//   _uart.Fail_PSC=RESET;
////   bMemory_W_TimeOut=RESET;
////   bMemory_R_TimeOut=RESET;
////   _uart.sensor1=RESET;
////   bSimmer_check=RESET;
////   bWater_Temp=RESET;
//
//   Contactor_Con(ON);
//
//   CH_LEFT=STANDBY_MODE;
//}

//void Auto_Response(void)
//{
//   if(!SF1.bAutoResopnse)
//   {
////      if((bSYSTEM_Temp)   && (_uart.SendAutoR == 1)) Temperature();
////      if(RF.bSYSTEM_Sensor && (_uart.SendAutoR == 2)) SEND_STATUS();
////      if((bLP3000_Status) && (_uart.SendAutoR == 3)) SEND_PowerStatus();
//      SF1.bAutoResopnse=SET;
//   }
//}

//unsigned IGBT_Temperature(unsigned int temp)
//{
//  unsigned int data=0;
//
//  if(temp> 1023) data=0;
//  else if(temp>= 510) data=1070- temp;
//  else if(temp< 510) data=1210- ((temp* 128)/ 100);
//
//  return data;
//}

void Encoder_Check()
{
  uint8_t MSB, LSB;
  uint32_t encoded, sum;

  if((_io_CHK._bJogPushSW_CNT>= ChkTime_10ms)&& !Encoder_SW_flag)
  {
    Encoder_SW_flag=1;
    SEND_1BYTE(CMD_JOG, PRESSED);
    if(use_debug4) DBG_PRINT("Button pressed\r\n");
  }

  MSB=LL_GPIO_IsInputPinSet(Jog_encoder_A_GPIO_Port, Jog_encoder_A_Pin); //MSB = most significant bit
  LSB=LL_GPIO_IsInputPinSet(Jog_encoder_B_GPIO_Port, Jog_encoder_B_Pin); //LSB = least significant bit

  encoded=(MSB<< 1)| LSB;  //converting the 2 pin value to single number
  sum=(lastEncoded<< 2)| encoded;   //adding it to the previous encoded value

  //if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
  if(sum== 0x0d|| sum== 0x04|| sum== 0x02|| sum== 0x0b)
  {
    encoderValue++;
    if(encoderValue>= 2)
    {
      encoderValue=0;
      SEND_1BYTE(CMD_JOG, ROTATE_CW);
      if(use_debug4) DBG_PRINT("CW\r\n");
    }
  }

  //if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
  if(sum== 0x0e|| sum== 0x07|| sum== 0x01|| sum== 0x08)
  {
    encoderValue--;
    if(encoderValue<= -2)
    {
      encoderValue=0;
      SEND_1BYTE(CMD_JOG, ROTATE_CCW);
      if(use_debug4) DBG_PRINT("CCW\r\n");
    }
  }

  lastEncoded=encoded; //store this value for next time

  encoder_flag=0;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
#if 0
  uint8_t reset_cause[7]={0, };
  if(LL_RCC_IsActiveFlag_IWDGRST()) reset_cause[0]=1; // Independent Watchdog reset
  if(LL_RCC_IsActiveFlag_LPWRRST()) reset_cause[1]=1; // Low Power reset
  if(LL_RCC_IsActiveFlag_OBLRST()) reset_cause[2]=1;  // Option byte reset
  if(LL_RCC_IsActiveFlag_PINRST()) reset_cause[3]=1;  // Pin reset
  if(LL_RCC_IsActiveFlag_SFTRST()) reset_cause[4]=1;  // Software reset
  if(LL_RCC_IsActiveFlag_WWDGRST()) reset_cause[5]=1; // Window Watchdog reset
  if(LL_RCC_IsActiveFlag_BORRST()) reset_cause[6]=1;  // BOR reset
  LL_RCC_ClearResetFlags();
#endif
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  MX_ADC5_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
#if 0
  delay_ms(200);
  for(uint8_t z=0; z< 7; z++)
  {
    DBG_PRINT("reset_cause[%d]=%d\r\n", z, reset_cause[z]);
  }
#endif
  // TODO
  use_debug=0;
  use_debug2=0; // temp.
  use_debug3=0; // hall
  use_debug4=0; // jog

  LOG_PRINT("\n[MS19] SYSTEM TEST ver.201223-001\r\n");
  LOG_PRINT("-----------------------------------\r\n");

  LOG_PRINT("SMPS Remote ON\r\n");
  REMOTE_ON;

  LOG_PRINT("Power-Button LED ON\r\n");
  power_on_flag=1;

  BUZZER_ON;
  delay_ms(200);
  BUZZER_OFF;

  delay_ms(1000);
  if(LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_4))
  {
    LOG_PRINT("AC Contactor ON\r\n");
    CONTACTOR_ON;
  }

  LOG_PRINT("Variable Initialize\r\n");
  value_init();

  LOG_PRINT("All channel set standby\r\n");
  RunToStopA();
  RunToStopB();
  RunToStopVal();

  // Jog Backlight
  LOG_PRINT("JogDial Backlight ON\r\n");
  TIM2->CCR2=5000; // 100%
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  LOG_PRINT("System clock: %ld MHz\r\n", SystemCoreClock/ 1000000);

  LOG_PRINT("Wait 3s for the PSC to be ready...");
  delay_ms(3000);
  LOG_PRINT("DONE\r\n");

#ifdef USE_1WIRE_MEMORY
  LOG_PRINT("1-Wire Memory Check\r\n");
  DS2482_detect();
  mem_done=1;
#endif

#ifdef USE_ADC_CONV
  LOG_PRINT("ADC Calibration\r\n");
  Activate_ADC();
#endif

//  // Front LED
//  LOG_PRINT("Front LED ON\r\n");
//  FRONT_LED_ON;
//  TIM2->CCR1=250; // 5%
//  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // wait for GUI to finish booting
#if 1
  LOG_PRINT("Wait 5s for the GUI to be ready...");
  delay_ms(5000);
#else
  LOG_PRINT("Wait 20s for the GUI to be ready...");
  for(uint8_t i=0; i< 4; i++)
  {
    delay_ms(5000);
  }
#endif
  LOG_PRINT("DONE\r\n");

  /* Disable DMA Channel Tx */
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);

  HAL_TIM_Base_Start_IT(&htim6);  // High-Voltage pulse generator timer (period: 200us)
  HAL_TIM_Base_Start(&htim3);  // hall sensor ADC timer (period: 4us)
  HAL_TIM_Base_Start(&htim7);  // ADC1 & ADC5 timer (period: 1sec)

  //HAL_ADC_Start(&hadc1);
  if((LL_ADC_IsEnabled(ADC1)== 1)&& (LL_ADC_IsDisableOngoing(ADC1)== 0)&& (LL_ADC_REG_IsConversionOngoing(ADC1)== 0))
  {
    LL_ADC_REG_StartConversion(ADC1);
  }

  HAL_ADC_Start(&hadc5);
//  if((LL_ADC_IsEnabled(ADC5)== 1)&& (LL_ADC_IsDisableOngoing(ADC5)== 0)&& (LL_ADC_REG_IsConversionOngoing(ADC5)== 0))
//  {
//    LL_ADC_REG_StartConversion(ADC5);
//  }

  LOG_PRINT("-- Booting done --\r\n\n");
  power_on_flag=0;
  PUSH_LED_ON;

  countTemp=3;
  check15sec=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
    if(!commErr)
    {
      if(GUI_Boot_OK&& check15sec)
      {
        check15sec=0;
        if(countTemp!= 0)
        {
          commErr=SET;
          for(int z=0; z< 3; z++)
          {
            BUZZER_ON;
            delay_ms(200);
            BUZZER_OFF;
            delay_ms(100);
          }
        }
        else countTemp=3;
      }
      else if(check15sec) check15sec=0;
    }

    // TODO
#ifdef USE_ADC_CONV
    if(adc5Done)
    {
      adc5Done=0;
      if(adc5Val[1]>= 100) EF2.bError28=SET;
      else
      {
        if(adc5Val[1]<= 34) uLimit=1200;
        else uLimit=adc5Val[1]* 35;
      }

      if(use_debug2)
      {
        DBG_PRINT("V_Vref=%d\r\n", adc5Val[0]);
        DBG_PRINT("V_TS=%d\r\n", adc5Val[1]);
        DBG_PRINT("uLimit=%d\r\n", uLimit);
      }
    }

    if(adc1Done)
    {
      adc1Done=0;
      if(use_debug2)
      {
#if 0
        double ax=adc1Val[0];
        //double ay=(2e-9* ax* ax* ax)- (1e-5* ax* ax)+ (0.0483* ax)- 18.789;
        double ay=35.16* log(ax)- 225.32;
        double bx=adc1Val[1];
        //double by=(2e-9* bx* bx* bx)- (1e-5* bx* bx)+ (0.0483* bx)- 18.789;
        double by=35.16* log(bx)- 225.32;
        DBG_PRINT("temp_chA=%d, %.1f\r\n", adc1Val[0], ay);
        DBG_PRINT("temp_chB=%d, %.1f\r\n", adc1Val[1], by);
#else
        DBG_PRINT("temp_chA=%d\r\n", adc1Val[0]);
        DBG_PRINT("temp_chB=%d\r\n", adc1Val[1]);
#endif
        DBG_PRINT("\n");
      }

#if 1
      if(AP_A_Fan_Active)
      {
        if(adc1Val[0]<= uLimit) underTempA++;
        else
        {
          underTempA=0;
          AP_A_Temp=0;
        }

        if(underTempA> 15)
        {
          underTempA=15;
          if((CH_LEFT== STOP_MODE)|| (CH_LEFT== ERROR_MODE))
          {
            //AP_A_FAN_OFF();
            if(SF2.bStatus21== SET) SF2.bStatus21=RESET;
          }
          //else AP_A_Temp=1;
        }
      }

      if(AP_B_Fan_Active)
      {
        if(adc1Val[1]<= uLimit) underTempB++;
        else
        {
          underTempB=0;
          AP_B_Temp=0;
        }

        if(underTempB> 15)
        {
          underTempB=15;
          if((CH_RIGHT== STOP_MODE)|| (CH_RIGHT== ERROR_MODE))
          {
            //AP_B_FAN_OFF();
            if(SF2.bStatus22== SET) SF2.bStatus22=RESET;
          }
          //else AP_B_Temp=1;
        }
      }
#endif

#if 1
      if(CH_LEFT!= STOP_MODE)
      {
        if(adc1Val[0]>= 2200)
        {
          overTempA++;
          if(overTempA> 10)
          {
            overTempA=10;
            SF2.bStatus21=SET;
          }
        }
      }

      if(CH_RIGHT!= STOP_MODE)
      {
        if(adc1Val[1]>= 2200)
        {
          overTempB++;
          if(overTempB> 10)
          {
            overTempB=10;
            SF2.bStatus22=SET;
          }
        }
      }
#endif
    }

    if(hallA_cnt!= 0)
    {
      if(phase_3A_flag&& !intenA_Zero)
      {
        if(hallA_cnt> HALL_COUNT) hallA_cnt=HALL_COUNT;

        qsort(hallA_low, hallA_cnt, sizeof(uint16_t), compare);
        if((CH_RIGHT!= RUN_MODE)|| (pmr.mode== TWIST)) sendHallA=1;
        else clearHallA=1;
        //delay_ms(1);
      }
      else if(intenA_Zero)
      {
        for(int i=0; i< HALL_COUNT; i++)
        {
          hallA_low[i]=VDDA_APPLI;
        }
        hallA_cnt=0;
      }
    }

    if(hallB_cnt!= 0)
    {
      if(phase_3B_flag&& !intenB_Zero)
      {
        if(hallB_cnt> HALL_COUNT) hallB_cnt=HALL_COUNT;

        qsort(hallB_low, hallB_cnt, sizeof(uint16_t), compare);
        //if((CH_LEFT!= RUN_MODE) || (pmr.mode== TWIST))
        sendHallB=1;
      }
      else if(intenB_Zero)
      {
        for(int i=0; i< HALL_COUNT; i++)
        {
          hallB_low[i]=VDDA_APPLI;
        }
        hallB_cnt=0;
      }
    }

    if(sendHallA|| sendHallB)
    {
      if(operation_flag== 2)
      {
        SEND_HALL(pmr.mode, VDDA_APPLI- hallA_low[1], VDDA_APPLI- hallB_low[1]);
        if(use_debug3) DBG_PRINT("minA=%ld  minB=%ld\r\n\n", VDDA_APPLI- hallA_low[1], VDDA_APPLI- hallB_low[1]);
      }

      if(sendHallA|| clearHallA)
      {
        sendHallA=0;
        clearHallA=0;
        for(int i=0; i< HALL_COUNT; i++)
        {
          hallA_low[i]=VDDA_APPLI;
        }
        hallA_cnt=0;
      }

      if(sendHallB)
      {
        sendHallB=0;
        for(int i=0; i< HALL_COUNT; i++)
        {
          hallB_low[i]=VDDA_APPLI;
        }
        hallB_cnt=0;
      }
    }
#endif

    if(encoder_flag)
    {
      Encoder_Check();
      if(!veryfirst_boot) Pwr_Btn_Check();
    }

    if(DA_update_A_flag)
    {
      DA_update_A_flag=0;
      POWER_DA_CONTROL_A(chA_DA);

      if(phase_4A_flag&& once_A_flag)
      {
        Ready_A_Stop();
        if((_master_flag== CHANNEL_L)&& _twist_once_)
        {
          _twist_once_=0;
          Ready_B_Start();
        }
        once_A_flag=0;
      }
    }

    if(DA_update_B_flag)
    {
      DA_update_B_flag=0;
      POWER_DA_CONTROL_B(chB_DA);

      if(phase_4B_flag&& once_B_flag)
      {
        Ready_B_Stop();
        if((_master_flag== CHANNEL_R)&& _twist_once_)
        {
          _twist_once_=0;
          Ready_A_Start();
        }
        once_B_flag=0;
      }
    }

    if(PSC_VerCheck1== 2&& PSC_VerCheck2== 2)
    {
      PSC_VerCheck1=0;
      PSC_VerCheck2=0;
      SEND_VERSION();
      ver_count=0;
    }

    //---------------------
    // GUI Command Process
    //---------------------

    if(GUI_CMD!= RESET)
    {
//      if(GUI_CMD== CMD_RUN)
//      {
//        SEND_1BYTE(CMD_RUN, _uart.reply);
//      }
//      else
      if(GUI_CMD== CMD_VERSION)
      {
        _send_cmd3('V');
        PSC_VerCheck1=1;
        _send_cmd5('V');
        PSC_VerCheck2=1;
      }
      else if(GUI_CMD== CMD_GUI_ONOFF)
      {
        if(_uart.reply== 0)
        {
          LOG_PRINT("--- GUI booting OK ---\r\n");
          SEND_1BYTE(GUI_CMD, _uart.reply);

          GUI_Boot_OK=1;
          uwTick=0;
          delay_ms(10);
          if(waitingACK) SEND_1BYTE(CMD_GUI_ONOFF, 1);
          else SEND_STATUS();
        }
        else if(_uart.reply== 1)
        {
//          uint8_t chA=0, chB=0;
//
//          /* Stop channel 1 (Q-pulse-A) */
//          HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//          TIM1->CNT=0;
//
//          /* Stop channel 1 (Q-pulse-B) */
//          HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
//          TIM8->CNT=0;
//
//          if(CH_LEFT!= STANDBY_MODE) chA=CHANNEL_L;
//          if(CH_RIGHT!= STANDBY_MODE) chB=CHANNEL_R;
//
//          Ready2Standby(chA+ chB);
//
//          operation_flag=0;
          waitingACK_done=1;
          LOG_PRINT("DONE\r\n");
        }
      }
      //TODO
      else if(GUI_CMD== CMD_PARAMETER)
      {
        if(operation_flag== 2)
        {
          if(_uart.reply== 1)
          {
            uStep=(pmr.uTime* 100)/ (1000/ pmr.freq0);
//            if(use_debug) DBG_PRINT("uStep=%ld\r\n", uStep);

            dStep=(pmr.dTime* 100)/ (1000/ pmr.freq0);
//            if(use_debug) DBG_PRINT("dStep=%ld\r\n", dStep);
          }
          else if(_uart.reply== 2)
          {
            if(pmr.intenCh== 0)
            {
              if(!SF1.bStatus11)
              {
                // Only when starting from the very first stop
                // Use "CHANGE_MODE" to give an exception
                if((CH_LEFT== STOP_MODE)|| (CH_LEFT== PAUSE_MODE)|| chA_DA_zero) chA_DA=0;

                if(pmr.freq0!= 0)
                {
                  pmr.intenA=pmr.intenA_new;

                  if(uStep> 0)
                  {
                    //if((pmr.intenA!= 0)&& (dischargeA== 0))
                    if(pmr.intenA!= 0)
                    {
                      // Use "CHANGE_MODE" to give an exception
                      if((CH_LEFT== STOP_MODE)|| (CH_LEFT== PAUSE_MODE)|| chA_DA_zero)
                      {
                        if(chA_DA_zero) chA_DA_zero=0;
                        chA_uStep=pmr.intenA/ uStep;
                        reaction_A_flag=1;
                      }
                      else
                      {
                        if(pmr.intenA> CONSTANT_DA) chA_uStep=(pmr.intenA- CONSTANT_DA)/ uStep;
                        else chA_uStep=0;
                      }

                      if(dStep> 0)
                      {
                        if(pmr.intenA> CONSTANT_DA) chA_dStep=(pmr.intenA- CONSTANT_DA)/ dStep;
                        else chA_dStep=0;
                      }

                      if(CH_LEFT!= RUN_MODE)
                      {
                        phase_1A_flag=1;
                        phase_1A_count=0;
                        phase_2A_flag=0;
                        phase_2A_count=0;
                        phase_3A_flag=0;
                        phase_3A_count=0;
                        phase_4A_flag=0;
                        phase_4A_count=0;
                        once_A_flag=0;

                        if(_master_flag== 0) _master_flag=CHANNEL_L;

                        if(pmr.mode!= TWIST)
                        {
                          if((CH_LEFT== CHANGE_MODE)|| (CH_LEFT== STOP_MODE)|| (CH_LEFT== PAUSE_MODE))
                          {
                            if(_master_flag== CHANNEL_L)
                            {
                              if((pmr.linked)|| (CH_RIGHT== REST_MODE))
                              {
                                pending_flag=1;
                                if(use_debug) DBG_PRINT("ch_L is pending\r\n");
                              }
                              else Ready_A_Start();
                            }
                            else  // when ch.R is master
                            {
                              if(pending_flag)
                              {
                                pending_flag=0;
                                Ready_B_Start();
                                Ready_A_Start();
                                if(pmr.linked) pmr.linked=0;
                              }
                              else
                              {
                                _next_step_=1;
                                if(use_debug) DBG_PRINT("ch_L is _next_step_\r\n");
                              }
                            }
                          }
                          else
                          {
                            if(use_debug) DBG_PRINT("ch_L is Freezing\r\n");
                          }
                        }
                        else
                        {
                          if(_master_flag== CHANNEL_L) Ready_A_Start();
                          else
                          {
                            _twist_once_=1;
                            if(use_debug) DBG_PRINT("ch_L is _twist_once_\r\n");
                          }
                        }
                        AP_A_FAN_ON();
                        old_CH_L=CH_LEFT;
                        CH_LEFT=RUN_MODE;
                        if(use_debug) DBG_PRINT("CH_LEFT=RUN_MODE\r\n");
                      }
                    }
                    else
                    {
                      if(pending_flag)
                      {
                        pending_flag=0;
                        Ready_B_Start();
                      }
                    }
                  }
                  else
                  {
                    if(pending_flag)
                    {
                      pending_flag=0;
                      Ready_B_Start();
                    }
                    RunToPause(CHANNEL_L);
                    if(use_debug) DBG_PRINT("uStep is 0\r\n");
                  }
                }
                else
                {
                  if(pending_flag)
                  {
                    pending_flag=0;
                    Ready_B_Start();
                  }
                  RunToPause(CHANNEL_L);
                  if(use_debug) DBG_PRINT("pmr.freq0 is 0\r\n");
                }
              }
              else
              {
                if(pending_flag)
                {
                  pending_flag=0;
                  Ready_B_Start();
                }
                RunToPause(CHANNEL_L);
                if(use_debug) DBG_PRINT("ch_L is disconnected!\r\n");
              }
            }
            else if(pmr.intenCh== 1)
            {
              if(!SF1.bStatus12)
              {
                // Only when starting from the very first stop
                // Use "CHANGE_MODE" to give an exception
                if((CH_RIGHT== STOP_MODE)|| (CH_RIGHT== PAUSE_MODE)|| chB_DA_zero) chB_DA=0;

                if(pmr.freq0!= 0)
                {
                  pmr.intenB=pmr.intenB_new;

                  if(uStep> 0)
                  {
                    //if((pmr.intenB!= 0)&& (dischargeB== 0))
                    if(pmr.intenB!= 0)
                    {
                      // Use "CHANGE_MODE" to give an exception
                      if((CH_RIGHT== STOP_MODE)|| (CH_RIGHT== PAUSE_MODE)|| chB_DA_zero)
                      {
                        if(chB_DA_zero) chB_DA_zero=0;
                        chB_uStep=pmr.intenB/ uStep;
                        reaction_B_flag=1;
                      }
                      else
                      {
                        if(pmr.intenB> CONSTANT_DA) chB_uStep=(pmr.intenB- CONSTANT_DA)/ uStep;
                        else chB_uStep=0;
                      }

                      if(dStep> 0)
                      {
                        if(pmr.intenB> CONSTANT_DA) chB_dStep=(pmr.intenB- CONSTANT_DA)/ dStep;
                        else chB_dStep=0;
                      }

                      if(CH_RIGHT!= RUN_MODE)
                      {
                        phase_1B_flag=1;
                        phase_1B_count=0;
                        phase_2B_flag=0;
                        phase_2B_count=0;
                        phase_3B_flag=0;
                        phase_3B_count=0;
                        phase_4B_flag=0;
                        phase_4B_count=0;
                        once_B_flag=0;

                        if(_master_flag== 0) _master_flag=CHANNEL_R;

                        if(pmr.mode!= TWIST)
                        {
                          if((CH_RIGHT== CHANGE_MODE)|| (CH_RIGHT== STOP_MODE)|| (CH_RIGHT== PAUSE_MODE))
                          {
                            if(_master_flag== CHANNEL_R)
                            {
                              if((pmr.linked)|| (CH_LEFT== REST_MODE))
                              {
                                pending_flag=1;
                                if(use_debug) DBG_PRINT("ch_R is pending\r\n");
                              }
                              else Ready_B_Start();
                            }
                            else  // when ch.A is master
                            {
                              if(pending_flag)
                              {
                                pending_flag=0;
                                Ready_A_Start();
                                Ready_B_Start();
                                if(pmr.linked) pmr.linked=0;
                              }
                              else
                              {
                                _next_step_=1;
                                if(use_debug) DBG_PRINT("ch_R is _next_step_\r\n");
                              }
                            }
                          }
                          else
                          {
                            if(use_debug) DBG_PRINT("ch_R is Freezing\r\n");
                          }
                        }
                        else
                        {
                          if(_master_flag== CHANNEL_R) Ready_B_Start();
                          else
                          {
                            _twist_once_=1;
                            if(use_debug) DBG_PRINT("ch_R is _twist_once_\r\n");
                          }
                        }
                        AP_B_FAN_ON();
                        old_CH_R=CH_RIGHT;
                        CH_RIGHT=RUN_MODE;
                        if(use_debug) DBG_PRINT("CH_RIGHT=RUN_MODE\r\n");
                      }
                    }
                    else
                    {
                      if(pending_flag)
                      {
                        pending_flag=0;
                        Ready_A_Start();
                      }
                    }
                  }
                  else
                  {
                    if(pending_flag)
                    {
                      pending_flag=0;
                      Ready_A_Start();
                    }
                    RunToPause(CHANNEL_R);
                    if(use_debug) DBG_PRINT("chB_step is 0\r\n");
                  }
                }
                else
                {
                  if(pending_flag)
                  {
                    pending_flag=0;
                    Ready_A_Start();
                  }
                  RunToPause(CHANNEL_R);
                  if(use_debug) DBG_PRINT("pmr.freqB is 0\r\n");
                }
              }
              else
              {
                if(pending_flag)
                {
                  pending_flag=0;
                  Ready_A_Start();
                }
                RunToPause(CHANNEL_R);
                if(use_debug) DBG_PRINT("ch_R is disconnected!\r\n");
              }
            }
          }
        }
        else
        {
          if(use_debug) DBG_PRINT("-deactivated-\r\n");
        }

        SEND_1BYTE(CMD_PARAMETER, _uart.reply);
      }
#ifdef USE_1WIRE_MEMORY
      else if(GUI_CMD== CMD_MEMORY)
      {
        uwTick=0;
        if(_uart.reply== 0) // save
        {
          int32_t retVal=0;
          uint8_t sendValue[16]={0, };

          sendValue[0]=WATERMARK;
          sendValue[1]=_memory.ch;
          sendValue[2]=_memory.id;
          sendValue[3]=_memory.len;
          for(int i=0; i< _memory.len; i++)
          {
            sendValue[i+ 4]=_memory.asc[i];
          }
          retVal=OW_Copy_Scratchpad(_memory.loc, sendValue);
          mem_done=1;
          SEND_MEMREPLY(retVal);

          if(retVal== false)
          {
            if(use_debug) DBG_PRINT("memory %d save fail\r\n", _memory.loc);

            if(_memory.loc== 0) EF4.bError41=SET;
            else if(_memory.loc== 1) EF4.bError42=SET;
          }
          else
          {
            if(use_debug) DBG_PRINT("memory %d save done\r\n", _memory.loc);
          }
        }
        else if(_uart.reply== 1)  // read
        {
          uint8_t *readValue;

          memRead_flag=true;
          readValue=OW_Read_Memory(_memory.loc);
          mem_done=1;

          if(memRead_flag== false)
          {
            for(memRead_cnt=0; memRead_cnt< 3; memRead_cnt++)
            {
              mem_done=0;
              delay_ms(100);
              memRead_flag=true;
              readValue=OW_Read_Memory(_memory.loc);
              mem_done=1;

              if(memRead_flag== false)
              {
                if(memRead_cnt== 2)
                {
                  // error 처리
                  SEND_MEMREPLY(0);
                  if(use_debug) DBG_PRINT("Memory may be unconnected or faulty.\r\n");

                  if(_memory.loc== 0) EF4.bError41=SET;
                  else if(_memory.loc== 1) EF4.bError42=SET;
                }
              }
              else
              {
                _memory.len=readValue[3];
                if(_memory.len> 10) _memory.len=10;

                SEND_MEMREAD(readValue);
                if(use_debug) DBG_PRINT("memory %d read done\r\n", _memory.loc);
                break;
              }
            }
          }
          else
          {
            if(!((readValue[1]== 1)|| (readValue[1]== 2)))
            {
              // error 처리
              SEND_MEMREPLY(1);
              if(use_debug) DBG_PRINT("Memory data may be anomaly_0.\r\n");

              if(_memory.loc== 0) EF4.bError43=SET;
              else if(_memory.loc== 1) EF4.bError44=SET;
            }
            /*else if(readValue[2]!= 1)
             {
             // error 처리
             SEND_MEMREPLY(1);
             if(use_debug) DBG_PRINT("Memory data may be anomaly_1.\r\n");

             if(_memory.loc== 0) EF4.bError43=SET;
             else if(_memory.loc== 1) EF4.bError44=SET;
             }*/
            else
            {
              _memory.len=readValue[3];
              if(_memory.len> 10) _memory.len=10;

              SEND_MEMREAD(readValue);
              if(use_debug) DBG_PRINT("memory %d read done\r\n", _memory.loc);
            }
          }
        }
      }
#endif
      else if(GUI_CMD== CMD_TEMPER)
      {
        SEND_TEMP(adc1Val[0], adc1Val[1], adc5Val[1]);
      }
      else if(GUI_CMD== CMD_undefine)
      {
        SEND_1BYTE(CMD_undefine, _uart.reply);
      }

      GUI_CMD=RESET;
    }

    //--------------------------------
    // Power(PSC) Command Process
    //--------------------------------
    if(POWER_CMD3!= RESET)
    {
      if(POWER_CMD3== p_Version)
      {
        PSC_Version1=byte2int(rbuf3[5], rbuf3[4]);

        if((rbuf3[7]== 110)|| (rbuf3[7]== 220)) AC_VoltBuffer1=rbuf3[7];
        //else AC_VoltBuffer1=220;

        if(PSC_VerCheck1== 1) PSC_VerCheck1=2;

        if(use_debug) DBG_PRINT("PSC1 ver=%ld ACvolt=%ld\r\n", PSC_Version1, AC_VoltBuffer1);
      }
      else if(POWER_CMD3== p_Error)
      {
        if(rbuf3[4]!= 0)
        {
          if(use_debug) DBG_PRINT("\nPSC1 error=%d\r\n\n", rbuf3[4]);
        }
      }
      /*else if(POWER_CMD3== p_Status)
       {
       if(rbuf3[4]== 0)
       {
       USINT _iChargeV=byte2int(rbuf3[6], rbuf3[5]);
       if(use_debug) DBG_PRINT("\nPSC1 charge volt=%d\r\n\n", _iChargeV);
       }
       else if(rbuf3[4]== 1)
       {
       USINT _iDCLinkV=byte2int(rbuf3[6], rbuf3[5]);
       if(use_debug) DBG_PRINT("\nPSC1 DC link=%d\r\n\n", _iDCLinkV);
       }
       else if(rbuf3[4]== 2)
       {
       USINT _iGIBTTemp=IGBT_Temperature(byte2int(rbuf3[6], rbuf3[5]));
       if(use_debug) DBG_PRINT("\nPSC1 IGBT temp=%d\r\n\n", _iGIBTTemp);
       }
       }*/
      POWER_CMD3=RESET;

      LL_USART_TransmitData8(USART3, ACK);
      memset(rbuf3, 0, sizeof(rbuf3));
    }

    if(POWER_CMD5!= RESET)
    {
      if(POWER_CMD5== p_Version)
      {
        PSC_Version2=byte2int(rbuf5[5], rbuf5[4]);

        if((rbuf5[7]== 110)|| (rbuf5[7]== 220)) AC_VoltBuffer2=rbuf5[7];
        //else AC_VoltBuffer2=220;

        if(PSC_VerCheck2== 1) PSC_VerCheck2=2;

        if(use_debug) DBG_PRINT("PSC2 ver=%ld ACvolt=%ld\r\n", PSC_Version2, AC_VoltBuffer2);
      }
      else if(POWER_CMD5== p_Error)
      {
        if(rbuf5[4]!= 0)
        {
          if(use_debug) DBG_PRINT("\nPSC2 error=%d\r\n\n", rbuf5[4]);
        }
      }
      /*else if(POWER_CMD5== p_Status)
       {
       if(rbuf5[4]== 0)
       {
       USINT _iChargeV=byte2int(rbuf5[6], rbuf5[5]);
       if(use_debug) DBG_PRINT("\nPSC2 charge volt=%d\r\n\n", _iChargeV);
       }
       else if(rbuf5[4]== 1)
       {
       USINT _iDCLinkV=byte2int(rbuf5[6], rbuf5[5]);
       if(use_debug) DBG_PRINT("\nPSC2 DC link=%d\r\n\n", _iDCLinkV);
       }
       else if(rbuf5[4]== 2)
       {
       USINT _iGIBTTemp=IGBT_Temperature(byte2int(rbuf5[6], rbuf5[5]));
       if(use_debug) DBG_PRINT("\nPSC2 IGBT temp=%d\r\n\n", _iGIBTTemp);
       }
       }*/
      POWER_CMD5=RESET;

      LL_USART_TransmitData8(UART5, ACK);
      memset(rbuf5, 0, sizeof(rbuf5));
    }

    // Status & Error Process
    if(_io_CHK._bCheckDelay>= ChkTime_500ms)
    {
      _io_CHK._bCheckDelay=RESET;
      Status_Check();
      Error_Check();
    }

    // Mode Change Send Data
    if(_io_CHK._OldModeA!= CH_LEFT)
    {
      BUZZER_OFF;

      if(CH_LEFT== ERROR_MODE)
      {
        Error_Sound_cnt();
        //SEND_ERROR();
        delay_ms(10);
        _io_CHK._BuzzerTIME=RESET;
      }

      _io_CHK._OldModeA=CH_LEFT;
    }

    if(_io_CHK._OldModeB!= CH_RIGHT)
    {
      BUZZER_OFF;

      if(CH_RIGHT== ERROR_MODE)
      {
        Error_Sound_cnt();
        //SEND_ERROR();
        delay_ms(10);
        _io_CHK._BuzzerTIME=RESET;
      }

      _io_CHK._OldModeB=CH_RIGHT;
    }

    if((CH_LEFT== ERROR_MODE)|| (CH_RIGHT== ERROR_MODE))
    {
      Pwr_Btn_Check();

      if(_io_CHK._BuzzerCNT!= 0)
      {
        uint8_t buzzer_count;

        //if(SF1.bBuzzerRefresh) buzzer_count=_io_CHK._BuzzerCNT;
        if(bBuzzerRefresh) buzzer_count=_io_CHK._BuzzerCNT;

        if(buzzer_count> 0)
        {
          //SF1.bBuzzerRefresh=RESET;
          bBuzzerRefresh=RESET;

          if(_io_CHK._BuzzerCNT< 11)
          {
            if(_io_CHK._BuzzerTIME< 50) HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
            else if((_io_CHK._BuzzerTIME> 50)&& (_io_CHK._BuzzerTIME< 250)) HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
            else if(_io_CHK._BuzzerTIME> 250)
            {
              HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
              if(buzzer_count> 0) buzzer_count--;
              _io_CHK._BuzzerTIME=RESET;
            }
          }
          else
          {
            if(_io_CHK._BuzzerTIME< 250) HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
            else if((_io_CHK._BuzzerTIME> 250)&& (_io_CHK._BuzzerTIME< 500)) HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
            else if(_io_CHK._BuzzerTIME> 500)
            {
              HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
              if(buzzer_count> 0) buzzer_count--;
              _io_CHK._BuzzerTIME=RESET;
            }
          }
        }
        else if(buzzer_count== 0)
        {
          if(_io_CHK._BuzzerTIME> 1000)
          {
            //SF1.bBuzzerRefresh=SET;
            bBuzzerRefresh=SET;

            _io_CHK._BuzzerTIME=RESET;
          }
        }
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();
  /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady()!= 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_6, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_Enable();
  /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady()!= 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
  /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource()!= LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1?s transition state at intermediate medium speed clock based on DWT */
  CoreDebug->DEMCR|= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL|= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT=0;
  while(DWT->CYCCNT< 100)
    ;
  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(170000000);

  /* Update the time base */
  if(HAL_InitTick(TICK_INT_PRIORITY)!= HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
  LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);
  LL_RCC_SetUARTClockSource(LL_RCC_UART4_CLKSOURCE_PCLK1);
  LL_RCC_SetUARTClockSource(LL_RCC_UART5_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C3_CLKSOURCE_PCLK1);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSOURCE_SYSCLK);
  LL_RCC_SetADCClockSource(LL_RCC_ADC345_CLKSOURCE_SYSCLK);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct={0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct={0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct={0};

  LL_GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
   PA0   ------> ADC1_IN1
   PA1   ------> ADC1_IN2
   */
  GPIO_InitStruct.Pin= AP_A_TEMP_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_A_TEMP_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin= AP_B_TEMP_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_B_TEMP_GPIO_Port, &GPIO_InitStruct);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
   */
  ADC_InitStruct.Resolution= LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment= LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode= LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource= LL_ADC_REG_TRIG_EXT_TIM7_TRGO;
  ADC_REG_InitStruct.SequencerLength= LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont= LL_ADC_REG_SEQ_DISCONT_1RANK;
  ADC_REG_InitStruct.ContinuousMode= LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer= LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun= LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC1, 0);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_GRP_REGULAR_CONTINUED);
  LL_ADC_ConfigOverSamplingRatioShift(ADC1, LL_ADC_OVS_RATIO_256, LL_ADC_OVS_SHIFT_RIGHT_8);
  LL_ADC_SetOverSamplingDiscont(ADC1, LL_ADC_OVS_REG_CONT);
  ADC_CommonInitStruct.CommonClock= LL_ADC_CLOCK_ASYNC_DIV8;
  ADC_CommonInitStruct.Multimode= LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index=((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US* (SystemCoreClock/ (100000* 2)))/ 10);
  while(wait_loop_index!= 0)
  {
    wait_loop_index--;
  }
  /** Configure Regular Channel
   */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
  /** Configure Regular Channel
   */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */

  LL_ADC_EnableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  LL_ADC_EnableIT_OVR(ADC1);

  /* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct={0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct={0};

  LL_GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC12);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  /**ADC2 GPIO Configuration
   PC0   ------> ADC2_IN6
   PC1   ------> ADC2_IN7
   */
  GPIO_InitStruct.Pin= AP_A_HALL_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_A_HALL_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin= AP_B_HALL_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_B_HALL_GPIO_Port, &GPIO_InitStruct);

  /* ADC2 DMA Init */

  /* ADC2 Init */
  LL_DMA_SetPeriphRequest(DMA2, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_ADC2);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA2, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_VERYHIGH);

  LL_DMA_SetMode(DMA2, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* ADC2 interrupt Init */
  NVIC_SetPriority(ADC1_2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
  NVIC_EnableIRQ(ADC1_2_IRQn);

  /* USER CODE BEGIN ADC2_Init 1 */

  /* Configure the DMA functional parameters for reception */
  LL_DMA_ConfigAddresses(DMA2, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC2, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t)DMA_ADC2_Buf,
    LL_DMA_GetDataTransferDirection(DMA2, LL_DMA_CHANNEL_1));
  LL_DMA_SetDataLength(DMA2, LL_DMA_CHANNEL_1, ADC_COUNT* 2);

  /* Enable DMA transfer complete/error interrupts  */
  //LL_DMA_EnableIT_HT(DMA2, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA2, LL_DMA_CHANNEL_1);

  /* USER CODE END ADC2_Init 1 */
  /** Common config
   */
  ADC_InitStruct.Resolution= LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment= LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode= LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource= LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
  ADC_REG_InitStruct.SequencerLength= LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont= LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode= LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer= LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  ADC_REG_InitStruct.Overrun= LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC2, 0);
  LL_ADC_SetOverSamplingScope(ADC2, LL_ADC_OVS_DISABLE);
  LL_ADC_REG_SetTriggerEdge(ADC2, LL_ADC_REG_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC2);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC2);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index=((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US* (SystemCoreClock/ (100000* 2)))/ 10);
  while(wait_loop_index!= 0)
  {
    wait_loop_index--;
  }
  /** Configure Regular Channel
   */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);
  /** Configure Regular Channel
   */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_6CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC2_Init 2 */

  /* Enable DMA Channel */
  LL_DMA_EnableChannel(DMA2, LL_DMA_CHANNEL_1);

  /* USER CODE END ADC2_Init 2 */

}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct={0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct={0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct={0};

  LL_GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Peripheral clock enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_ADC345);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  /**ADC3 GPIO Configuration
   PE7   ------> ADC3_IN4
   PE8   ------> ADC3_IN6
   */
  GPIO_InitStruct.Pin= reserve1_temp_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(reserve1_temp_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin= reserve2_temp_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(reserve2_temp_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
   */
  ADC_InitStruct.Resolution= LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment= LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode= LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC3, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource= LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength= LL_ADC_REG_SEQ_SCAN_ENABLE_2RANKS;
  ADC_REG_InitStruct.SequencerDiscont= LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode= LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer= LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun= LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC3, &ADC_REG_InitStruct);
  LL_ADC_SetGainCompensation(ADC3, 0);
  LL_ADC_SetOverSamplingScope(ADC3, LL_ADC_OVS_DISABLE);
  ADC_CommonInitStruct.CommonClock= LL_ADC_CLOCK_ASYNC_DIV128;
  ADC_CommonInitStruct.Multimode= LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC3), &ADC_CommonInitStruct);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC3);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC3);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index=((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US* (SystemCoreClock/ (100000* 2)))/ 10);
  while(wait_loop_index!= 0)
  {
    wait_loop_index--;
  }
  /** Configure Regular Channel
   */
  LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_4);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC3, LL_ADC_CHANNEL_4, LL_ADC_SINGLE_ENDED);
  /** Configure Regular Channel
   */
  LL_ADC_REG_SetSequencerRanks(ADC3, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC3, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_640CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC3, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
 * @brief ADC5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC5_Init(void)
{

  /* USER CODE BEGIN ADC5_Init 0 */

  /* USER CODE END ADC5_Init 0 */

  ADC_ChannelConfTypeDef sConfig={0};

  /* USER CODE BEGIN ADC5_Init 1 */

  /* USER CODE END ADC5_Init 1 */
  /** Common config
   */
  hadc5.Instance= ADC5;
  hadc5.Init.ClockPrescaler= ADC_CLOCK_ASYNC_DIV128;
  hadc5.Init.Resolution= ADC_RESOLUTION_12B;
  hadc5.Init.DataAlign= ADC_DATAALIGN_RIGHT;
  hadc5.Init.GainCompensation=0;
  hadc5.Init.ScanConvMode= ADC_SCAN_ENABLE;
  hadc5.Init.EOCSelection= ADC_EOC_SINGLE_CONV;
  hadc5.Init.LowPowerAutoWait=DISABLE;
  hadc5.Init.ContinuousConvMode=DISABLE;
  hadc5.Init.NbrOfConversion=2;
  hadc5.Init.DiscontinuousConvMode=ENABLE;
  hadc5.Init.NbrOfDiscConversion=1;
  hadc5.Init.ExternalTrigConv= ADC_EXTERNALTRIG_T7_TRGO;
  hadc5.Init.ExternalTrigConvEdge= ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc5.Init.DMAContinuousRequests=DISABLE;
  hadc5.Init.Overrun= ADC_OVR_DATA_OVERWRITTEN;
  hadc5.Init.OversamplingMode=ENABLE;
  hadc5.Init.Oversampling.Ratio= ADC_OVERSAMPLING_RATIO_256;
  hadc5.Init.Oversampling.RightBitShift= ADC_RIGHTBITSHIFT_8;
  hadc5.Init.Oversampling.TriggeredMode= ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc5.Init.Oversampling.OversamplingStopReset= ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if(HAL_ADC_Init(&hadc5)!= HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
   */
  sConfig.Channel= ADC_CHANNEL_VREFINT;
  sConfig.Rank= ADC_REGULAR_RANK_1;
  sConfig.SamplingTime= ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff= ADC_SINGLE_ENDED;
  sConfig.OffsetNumber= ADC_OFFSET_NONE;
  sConfig.Offset=0;
  if(HAL_ADC_ConfigChannel(&hadc5, &sConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
   */
  sConfig.Channel= ADC_CHANNEL_TEMPSENSOR_ADC5;
  sConfig.Rank= ADC_REGULAR_RANK_2;
  if(HAL_ADC_ConfigChannel(&hadc5, &sConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC5_Init 2 */

  LL_ADC_EnableIT_EOC(ADC5);
  LL_ADC_DisableIT_EOS(ADC5);
  LL_ADC_EnableIT_OVR(ADC5);

  /* USER CODE END ADC5_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance= I2C3;
  hi2c3.Init.Timing=0x10802D9B;
  hi2c3.Init.OwnAddress1=0;
  hi2c3.Init.AddressingMode= I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode= I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2=0;
  hi2c3.Init.OwnAddress2Masks= I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode= I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode= I2C_NOSTRETCH_DISABLE;
  if(HAL_I2C_Init(&hi2c3)!= HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
   */
  if(HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)!= HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
   */
  if(HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance= LPUART1;
  hlpuart1.Init.BaudRate=230400;
  hlpuart1.Init.WordLength= UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits= UART_STOPBITS_1;
  hlpuart1.Init.Parity= UART_PARITY_NONE;
  hlpuart1.Init.Mode= UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl= UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling= UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler= UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit= UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_Init(&hlpuart1)!= HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8)!= HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8)!= HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_UARTEx_DisableFifoMode(&hlpuart1)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

#ifdef USE_LPUART
  if(HAL_UART_Receive_IT(&hlpuart1, LP_RX_Buffer, 1)!= HAL_OK)
  {
    Error_Handler();
  }
#endif

  /* USER CODE END LPUART1_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance= UART4;
  huart4.Init.BaudRate=230400;
  huart4.Init.WordLength= UART_WORDLENGTH_8B;
  huart4.Init.StopBits= UART_STOPBITS_1;
  huart4.Init.Parity= UART_PARITY_NONE;
  huart4.Init.Mode= UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl= UART_HWCONTROL_NONE;
  huart4.Init.OverSampling= UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling= UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler= UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit= UART_ADVFEATURE_NO_INIT;
  if(HAL_UART_Init(&huart4)!= HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8)!= HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8)!= HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_UARTEx_DisableFifoMode(&huart4)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

#ifndef USE_LPUART
  if(HAL_UART_Receive_IT(&huart4, RX4_Buffer, 1)!= HAL_OK)
  {
    Error_Handler();
  }
#endif

  /* USER CODE END UART4_Init 2 */

}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct={0};

  LL_GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  /**UART5 GPIO Configuration
   PC12   ------> UART5_TX
   PD2   ------> UART5_RX
   */
  GPIO_InitStruct.Pin= PWR2_TX_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_5;
  LL_GPIO_Init(PWR2_TX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin= PWR2_RX_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_5;
  LL_GPIO_Init(PWR2_RX_GPIO_Port, &GPIO_InitStruct);

  /* UART5 DMA Init */

  /* UART5_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_5, LL_DMAMUX_REQ_UART5_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_5, LL_DMA_MDATAALIGN_BYTE);

  /* UART5_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_6, LL_DMAMUX_REQ_UART5_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_6, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN UART5_Init 1 */

  /* Configure the DMA functional parameters for reception */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_5, LL_USART_DMA_GetRegAddr(UART5, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t)DMA_UART5_buf,
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_5));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_5, 1);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_6, (uint32_t)tbuf5, LL_USART_DMA_GetRegAddr(UART5, LL_USART_DMA_REG_DATA_TRANSMIT),
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_6));

  /* Enable DMA transfer complete/error interrupts  */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_5);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_6);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_6);

  /* USER CODE END UART5_Init 1 */
  USART_InitStruct.PrescalerValue= LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate=57600;
  USART_InitStruct.DataWidth= LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits= LL_USART_STOPBITS_1;
  USART_InitStruct.Parity= LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection= LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl= LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling= LL_USART_OVERSAMPLING_16;
  LL_USART_Init(UART5, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(UART5, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_ConfigAsyncMode(UART5);

  /* USER CODE BEGIN WKUPType UART5 */

  /* USER CODE END WKUPType UART5 */

  LL_USART_Enable(UART5);

  /* Polling UART5 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(UART5)))|| (!(LL_USART_IsActiveFlag_REACK(UART5))))
  {
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(UART5);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_5);

  /* USER CODE END UART5_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct={0};

  LL_GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
   PA9   ------> USART1_TX
   PA10   ------> USART1_RX
   */
  GPIO_InitStruct.Pin= GUI_TX_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_7;
  LL_GPIO_Init(GUI_TX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin= GUI_RX_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_7;
  LL_GPIO_Init(GUI_RX_GPIO_Port, &GPIO_InitStruct);

  /* USART1 DMA Init */

  /* USART1_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_USART1_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);

  /* USART1_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_USART1_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN USART1_Init 1 */

  /* Configure the DMA functional parameters for reception */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t)DMA_USART1_buf,
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2, (uint32_t)tbuf1, LL_USART_DMA_GetRegAddr(USART1, LL_USART_DMA_REG_DATA_TRANSMIT),
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2));

  /* Enable DMA transfer complete/error interrupts  */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_2);

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue= LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate=115200;
  USART_InitStruct.DataWidth= LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits= LL_USART_STOPBITS_1;
  USART_InitStruct.Parity= LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection= LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl= LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling= LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1)))|| (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART1);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  /* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct={0};

  LL_GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
   PA2   ------> USART2_TX
   PA3   ------> USART2_RX
   */
  GPIO_InitStruct.Pin= Rsrv1_tx_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_7;
  LL_GPIO_Init(Rsrv1_tx_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin= Rsrv1_rx_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_7;
  LL_GPIO_Init(Rsrv1_rx_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.PrescalerValue= LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate=115200;
  USART_InitStruct.DataWidth= LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits= LL_USART_STOPBITS_1;
  USART_InitStruct.Parity= LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection= LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl= LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling= LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART2);
  LL_USART_ConfigAsyncMode(USART2);

  /* USER CODE BEGIN WKUPType USART2 */

  /* USER CODE END WKUPType USART2 */

  LL_USART_Enable(USART2);

  /* Polling USART2 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART2)))|| (!(LL_USART_IsActiveFlag_REACK(USART2))))
  {
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct={0};

  LL_GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  /**USART3 GPIO Configuration
   PD8   ------> USART3_TX
   PD9   ------> USART3_RX
   */
  GPIO_InitStruct.Pin= PWR1_TX_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_7;
  LL_GPIO_Init(PWR1_TX_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin= PWR1_RX_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_7;
  LL_GPIO_Init(PWR1_RX_GPIO_Port, &GPIO_InitStruct);

  /* USART3 DMA Init */

  /* USART3_RX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_3, LL_DMAMUX_REQ_USART3_RX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);

  /* USART3_TX Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_4, LL_DMAMUX_REQ_USART3_TX);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* USER CODE BEGIN USART3_Init 1 */

  /* Configure the DMA functional parameters for reception */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_RECEIVE), (uint32_t)DMA_USART3_buf,
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, 1);

  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_4, (uint32_t)tbuf3, LL_USART_DMA_GetRegAddr(USART3, LL_USART_DMA_REG_DATA_TRANSMIT),
    LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4));

  /* Enable DMA transfer complete/error interrupts  */
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_4);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_4);

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.PrescalerValue= LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate=57600;
  USART_InitStruct.DataWidth= LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits= LL_USART_STOPBITS_1;
  USART_InitStruct.Parity= LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection= LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl= LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling= LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART3, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART3);
  LL_USART_ConfigAsyncMode(USART3);

  /* USER CODE BEGIN WKUPType USART3 */

  /* USER CODE END WKUPType USART3 */

  LL_USART_Enable(USART3);

  /* Polling USART3 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART3)))|| (!(LL_USART_IsActiveFlag_REACK(USART3))))
  {
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* Enable DMA RX Interrupt */
  LL_USART_EnableDMAReq_RX(USART3);

  /* Enable DMA Channel Rx */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);

  /* USER CODE END USART3_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct={0};

  LL_GPIO_InitTypeDef GPIO_InitStruct={0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
   PA5   ------> SPI1_SCK
   PA7   ------> SPI1_MOSI
   */
  GPIO_InitStruct.Pin= LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin= LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate= LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection= LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode= LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth= LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity= LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase= LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS= LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate= LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder= LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation= LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly=7;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* Enable SPI1 */
  LL_SPI_Enable(SPI1);

  /* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  uint8_t FREQ=30;
  uint32_t PRESCALER_VALUE=(SystemCoreClock/ (TIM_COUNTER_CONST* FREQ))- 1;
  uint32_t PERIOD_VALUE=TIM_COUNTER_CONST- 1;
  uint32_t PULSE_VALUE=(PERIOD_VALUE+ 1)* 2/ (1000/ FREQ);

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig={0};
  TIM_OC_InitTypeDef sConfigOC={0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig={0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance= TIM1;
  htim1.Init.Prescaler=PRESCALER_VALUE;
  htim1.Init.CounterMode= TIM_COUNTERMODE_UP;
  htim1.Init.Period=PERIOD_VALUE;
  htim1.Init.ClockDivision= TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter=0;
  htim1.Init.AutoReloadPreload= TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_PWM_Init(&htim1)!= HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger= TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2= TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode= TIM_MASTERSLAVEMODE_DISABLE;
  if(HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode= TIM_OCMODE_PWM1;
  sConfigOC.Pulse=PULSE_VALUE;
  sConfigOC.OCPolarity= TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity= TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode= TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState= TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState= TIM_OCNIDLESTATE_RESET;
  if(HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)!= HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode= TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode= TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel= TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime=0;
  sBreakDeadTimeConfig.BreakState= TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity= TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter=0;
  sBreakDeadTimeConfig.BreakAFMode= TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State= TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity= TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter=0;
  sBreakDeadTimeConfig.Break2AFMode= TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput= TIM_AUTOMATICOUTPUT_DISABLE;
  if(HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  uint32_t PRESCALER_VALUE=(SystemCoreClock/ TIM_COUNTER_CLOCK )- 1; // 170-1
  uint32_t PERIOD_VALUE=(TIM_COUNTER_CLOCK / 200)- 1; // 5000-1

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig={0};
  TIM_OC_InitTypeDef sConfigOC={0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance= TIM2;
  htim2.Init.Prescaler=PRESCALER_VALUE;
  htim2.Init.CounterMode= TIM_COUNTERMODE_UP;
  htim2.Init.Period=PERIOD_VALUE;
  htim2.Init.ClockDivision= TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload= TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_PWM_Init(&htim2)!= HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger= TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode= TIM_MASTERSLAVEMODE_DISABLE;
  if(HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode= TIM_OCMODE_PWM1;
  sConfigOC.Pulse=0;
  sConfigOC.OCPolarity= TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode= TIM_OCFAST_DISABLE;
  if(HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)!= HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse=0;
  sConfigOC.OCPolarity= TIM_OCPOLARITY_HIGH;
  if(HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  uint32_t PRESCALER_VALUE=68- 1;
  uint32_t PERIOD_VALUE=10- 1;

  /* USER CODE END TIM3_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig={0};
  TIM_MasterConfigTypeDef sMasterConfig={0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance= TIM3;
  htim3.Init.Prescaler=PRESCALER_VALUE;
  htim3.Init.CounterMode= TIM_COUNTERMODE_UP;
  htim3.Init.Period=PERIOD_VALUE;
  htim3.Init.ClockDivision= TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload= TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&htim3)!= HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode= TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger= TIM_TS_ITR0;
  if(HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger= TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode= TIM_MASTERSLAVEMODE_DISABLE;
  if(HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  uint32_t PRESCALER_VALUE=170- 1;
  uint32_t PERIOD_VALUE=200- 1;

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig={0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance= TIM6;
  htim6.Init.Prescaler=PRESCALER_VALUE;
  htim6.Init.CounterMode= TIM_COUNTERMODE_UP;
  htim6.Init.Period=PERIOD_VALUE;
  htim6.Init.AutoReloadPreload= TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&htim6)!= HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger= TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode= TIM_MASTERSLAVEMODE_DISABLE;
  if(HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  uint32_t PRESCALER_VALUE=17000- 1;
  uint32_t PERIOD_VALUE=10000- 1;

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig={0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance= TIM7;
  htim7.Init.Prescaler=PRESCALER_VALUE;
  htim7.Init.CounterMode= TIM_COUNTERMODE_UP;
  htim7.Init.Period=PERIOD_VALUE;
  htim7.Init.AutoReloadPreload= TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&htim7)!= HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger= TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode= TIM_MASTERSLAVEMODE_DISABLE;
  if(HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  uint8_t FREQ=30;
  uint32_t PRESCALER_VALUE=(SystemCoreClock/ (TIM_COUNTER_CONST* FREQ))- 1;
  uint32_t PERIOD_VALUE=TIM_COUNTER_CONST- 1;
  uint32_t PULSE_VALUE=(PERIOD_VALUE+ 1)* 2/ (1000/ FREQ);

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig={0};
  TIM_OC_InitTypeDef sConfigOC={0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig={0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance= TIM8;
  htim8.Init.Prescaler=PRESCALER_VALUE;
  htim8.Init.CounterMode= TIM_COUNTERMODE_UP;
  htim8.Init.Period=PERIOD_VALUE;
  htim8.Init.ClockDivision= TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter=0;
  htim8.Init.AutoReloadPreload= TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_PWM_Init(&htim8)!= HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger= TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2= TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode= TIM_MASTERSLAVEMODE_DISABLE;
  if(HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode= TIM_OCMODE_PWM1;
  sConfigOC.Pulse=PULSE_VALUE;
  sConfigOC.OCPolarity= TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity= TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode= TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState= TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState= TIM_OCNIDLESTATE_RESET;
  if(HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1)!= HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode= TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode= TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel= TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime=0;
  sBreakDeadTimeConfig.BreakState= TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity= TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter=0;
  sBreakDeadTimeConfig.BreakAFMode= TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State= TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity= TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter=0;
  sBreakDeadTimeConfig.Break2AFMode= TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput= TIM_AUTOMATICOUTPUT_DISABLE;
  if(HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)!= HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0));
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
  NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0));
  NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 6, 0));
  NVIC_EnableIRQ(DMA2_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct={0};
  LL_GPIO_InitTypeDef GPIO_InitStruct={0};

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOE);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOG);

  /**/
  LL_GPIO_ResetOutputPin(CONTACTER_CTL_GPIO_Port, CONTACTER_CTL_Pin);

  /**/
  LL_GPIO_ResetOutputPin(Pwr_Btn_LED_GPIO_Port, Pwr_Btn_LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(AP_A_FAN_ctl_GPIO_Port, AP_A_FAN_ctl_Pin);

  /**/
  LL_GPIO_ResetOutputPin(Front_LED_OnOff_GPIO_Port, Front_LED_OnOff_Pin);

  /**/
  LL_GPIO_ResetOutputPin(SYS_FAN_CTL_GPIO_Port, SYS_FAN_CTL_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(AP_B_LED1_GPIO_Port, AP_B_LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(AP_B_LED2_GPIO_Port, AP_B_LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(AP_B_LED3_GPIO_Port, AP_B_LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(AP_B_FAN_ctl_GPIO_Port, AP_B_FAN_ctl_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LD4_GPIO_Port, LD4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LD1_GPIO_Port, LD1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(PC_brd_SW_GPIO_Port, PC_brd_SW_Pin);

  /**/
  LL_GPIO_ResetOutputPin(AP_A_LED1_GPIO_Port, AP_A_LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(AP_A_LED2_GPIO_Port, AP_A_LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(AP_A_LED3_GPIO_Port, AP_A_LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);

  /**/
  LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin);

  /**/
  LL_GPIO_ResetOutputPin(REMOTE_DATA_GPIO_Port, REMOTE_DATA_Pin);

  /**/
  LL_GPIO_ResetOutputPin(REMOTE_CTL_GPIO_Port, REMOTE_CTL_Pin);

  /**/
  LL_GPIO_SetOutputPin(READY_A_GPIO_Port, READY_A_Pin);

  /**/
  LL_GPIO_SetOutputPin(READY_B_GPIO_Port, READY_B_Pin);

  /**/
  LL_GPIO_SetOutputPin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= CONTACTER_CTL_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(CONTACTER_CTL_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= Pwr_Btn_LED_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(Pwr_Btn_LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= Door_interlock_R_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(Door_interlock_R_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LD3_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_A_FAN_ctl_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_A_FAN_ctl_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= Jog_encoder_A_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(Jog_encoder_A_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= Door_interlock_L_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(Door_interlock_L_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= Front_LED_OnOff_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(Front_LED_OnOff_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= READY_A_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(READY_A_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= SYS_FAN_sense_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(SYS_FAN_sense_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= SYS_FAN_CTL_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(SYS_FAN_CTL_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= KEY_CHECK_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(KEY_CHECK_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_A_CONNECT_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_A_CONNECT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_B_CONNECT_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_B_CONNECT_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LD2_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_B_LED1_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_B_LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_B_LED2_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_B_LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_B_LED3_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_B_LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= READY_B_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(READY_B_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= Jog_encoder_B_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(Jog_encoder_B_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_B_FAN_ctl_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_B_FAN_ctl_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= Jog_PushButton_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(Jog_PushButton_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LD4_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LD1_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AC_STATE_CHECK_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AC_STATE_CHECK_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= PC_brd_SW_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(PC_brd_SW_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_A_LED1_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_A_LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_A_LED2_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_A_LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= AP_A_LED3_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(AP_A_LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= BUZZER_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= MCP4922_CS_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(MCP4922_CS_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= REMOTE_DATA_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(REMOTE_DATA_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= REMOTE_CTL_Pin;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed= LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType= LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(REMOTE_CTL_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin= LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode= LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull= LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE1);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE2);

  /**/
  EXTI_InitStruct.Line_0_31= LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand=ENABLE;
  EXTI_InitStruct.Mode= LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger= LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31= LL_EXTI_LINE_1;
  EXTI_InitStruct.LineCommand=ENABLE;
  EXTI_InitStruct.Mode= LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger= LL_EXTI_TRIGGER_RISING_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31= LL_EXTI_LINE_2;
  EXTI_InitStruct.LineCommand=ENABLE;
  EXTI_InitStruct.Mode= LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger= LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(Q_pulse_A_in_GPIO_Port, Q_pulse_A_in_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(Q_pulse_B_in_GPIO_Port, Q_pulse_B_in_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinPull(Patient_interlock_GPIO_Port, Patient_interlock_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(Q_pulse_A_in_GPIO_Port, Q_pulse_A_in_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(Q_pulse_B_in_GPIO_Port, Q_pulse_B_in_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinMode(Patient_interlock_GPIO_Port, Patient_interlock_Pin, LL_GPIO_MODE_INPUT);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 1));
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 1));
  NVIC_EnableIRQ(EXTI1_IRQn);
  NVIC_SetPriority(EXTI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0));
  NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

#ifdef USE_ADC_CONV
/**
 * @brief  Perform ADC activation procedure to make it ready to convert
 *         (ADC instance: ADC1).
 * @note   Operations:
 *         - ADC instance
 *           - Disable deep power down
 *           - Enable internal voltage regulator
 *           - Run ADC self calibration
 *           - Enable ADC
 *         - ADC group regular
 *           none: ADC conversion start-stop to be performed
 *                 after this function
 *         - ADC group injected
 *           none: ADC conversion start-stop to be performed
 *                 after this function
 * @param  None
 * @retval None
 */
void Activate_ADC()
{
  __IO uint32_t wait_loop_index=0U;
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 0U; /* Variable used for timeout management */
  #endif /* USE_TIMEOUT */

  /*## Operation on ADC hierarchical scope: ADC instance #####################*/

  /* Note: Hardware constraint (refer to description of the functions         */
  /*       below):                                                            */
  /*       On this STM32 serie, setting of these features is conditioned to   */
  /*       ADC state:                                                         */
  /*       ADC must be disabled.                                              */
  /* Note: In this example, all these checks are not necessary but are        */
  /*       implemented anyway to show the best practice usages                */
  /*       corresponding to reference manual procedure.                       */
  /*       Software can be optimized by removing some of these checks, if     */
  /*       they are not relevant considering previous settings and actions    */
  /*       in user application.                                               */
  if(LL_ADC_IsEnabled(ADC1)== 0)
  {
    /* Disable ADC deep power down (enabled by default after reset state) */
    LL_ADC_DisableDeepPowerDown(ADC1);

    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);

    /* Delay for ADC internal voltage regulator stabilization.                */
    /* Compute number of CPU cycles to wait for, from delay in us.            */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    /* Note: If system core clock frequency is below 200kHz, wait time        */
    /*       is only a few CPU processing cycles.                             */
    wait_loop_index=((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US* (SystemCoreClock/ (100000* 2)))/ 10);
    while(wait_loop_index!= 0)
    {
      wait_loop_index--;
    }

    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);

    /* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
        Timeout = ADC_CALIBRATION_TIMEOUT_MS;
        #endif /* USE_TIMEOUT */

    while(LL_ADC_IsCalibrationOnGoing(ADC1)!= 0)
    {
#if (USE_TIMEOUT == 1)
          /* Check Systick counter flag to decrement the time-out value */
          if (LL_SYSTICK_IsActiveCounterFlag())
          {
            if(Timeout-- == 0)
            {
            /* Time-out occurred. Set LED to blinking mode */
            LED_Blinking(LED_BLINK_ERROR);
            }
          }
        #endif /* USE_TIMEOUT */
    }

    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index=(ADC_DELAY_CALIB_ENABLE_CPU_CYCLES>> 1);
    while(wait_loop_index!= 0)
    {
      wait_loop_index--;
    }

    /* Enable ADC */
    LL_ADC_Enable(ADC1);

    /* Poll for ADC ready to convert */
#if (USE_TIMEOUT == 1)
        Timeout = ADC_ENABLE_TIMEOUT_MS;
        #endif /* USE_TIMEOUT */

    while(LL_ADC_IsActiveFlag_ADRDY(ADC1)== 0)
    {
#if (USE_TIMEOUT == 1)
          /* Check Systick counter flag to decrement the time-out value */
          if (LL_SYSTICK_IsActiveCounterFlag())
          {
            if(Timeout-- == 0)
            {
            /* Time-out occurred. Set LED to blinking mode */
            LED_Blinking(LED_BLINK_ERROR);
            }
          }
        #endif /* USE_TIMEOUT */
    }
  }

  if(LL_ADC_IsEnabled(ADC2)== 0)
  {
    /* Disable ADC deep power down (enabled by default after reset state) */
    LL_ADC_DisableDeepPowerDown(ADC2);

    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC2);

    /* Delay for ADC internal voltage regulator stabilization.                */
    /* Compute number of CPU cycles to wait for, from delay in us.            */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    /* Note: If system core clock frequency is below 200kHz, wait time        */
    /*       is only a few CPU processing cycles.                             */
    wait_loop_index=((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US* (SystemCoreClock/ (100000* 2)))/ 10);
    while(wait_loop_index!= 0)
    {
      wait_loop_index--;
    }

    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);

    /* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
      Timeout = ADC_CALIBRATION_TIMEOUT_MS;
      #endif /* USE_TIMEOUT */

    while(LL_ADC_IsCalibrationOnGoing(ADC2)!= 0)
    {
#if (USE_TIMEOUT == 1)
        /* Check Systick counter flag to decrement the time-out value */
        if (LL_SYSTICK_IsActiveCounterFlag())
        {
          if(Timeout-- == 0)
          {
          /* Time-out occurred. Set LED to blinking mode */
          LED_Blinking(LED_BLINK_ERROR);
          }
        }
      #endif /* USE_TIMEOUT */
    }

    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index=(ADC_DELAY_CALIB_ENABLE_CPU_CYCLES>> 1);
    while(wait_loop_index!= 0)
    {
      wait_loop_index--;
    }

    /* Enable ADC */
    LL_ADC_Enable(ADC2);

    /* Poll for ADC ready to convert */
#if (USE_TIMEOUT == 1)
      Timeout = ADC_ENABLE_TIMEOUT_MS;
      #endif /* USE_TIMEOUT */

    while(LL_ADC_IsActiveFlag_ADRDY(ADC2)== 0)
    {
#if (USE_TIMEOUT == 1)
        /* Check Systick counter flag to decrement the time-out value */
        if (LL_SYSTICK_IsActiveCounterFlag())
        {
          if(Timeout-- == 0)
          {
          /* Time-out occurred. Set LED to blinking mode */
          LED_Blinking(LED_BLINK_ERROR);
          }
        }
      #endif /* USE_TIMEOUT */
    }
  }

  if(LL_ADC_IsEnabled(ADC5)== 0)
  {
    /* Disable ADC deep power down (enabled by default after reset state) */
    LL_ADC_DisableDeepPowerDown(ADC5);

    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC5);

    /* Delay for ADC internal voltage regulator stabilization.                */
    /* Compute number of CPU cycles to wait for, from delay in us.            */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    /* Note: If system core clock frequency is below 200kHz, wait time        */
    /*       is only a few CPU processing cycles.                             */
    wait_loop_index=((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US* (SystemCoreClock/ (100000* 2)))/ 10);
    while(wait_loop_index!= 0)
    {
      wait_loop_index--;
    }

    /* Run ADC self calibration */
    LL_ADC_StartCalibration(ADC5, LL_ADC_SINGLE_ENDED);

    /* Poll for ADC effectively calibrated */
#if (USE_TIMEOUT == 1)
        Timeout = ADC_CALIBRATION_TIMEOUT_MS;
        #endif /* USE_TIMEOUT */

    while(LL_ADC_IsCalibrationOnGoing(ADC5)!= 0)
    {
#if (USE_TIMEOUT == 1)
          /* Check Systick counter flag to decrement the time-out value */
          if (LL_SYSTICK_IsActiveCounterFlag())
          {
            if(Timeout-- == 0)
            {
            /* Time-out occurred. Set LED to blinking mode */
            LED_Blinking(LED_BLINK_ERROR);
            }
          }
        #endif /* USE_TIMEOUT */
    }

    /* Delay between ADC end of calibration and ADC enable.                   */
    /* Note: Variable divided by 2 to compensate partially                    */
    /*       CPU processing cycles (depends on compilation optimization).     */
    wait_loop_index=(ADC_DELAY_CALIB_ENABLE_CPU_CYCLES>> 1);
    while(wait_loop_index!= 0)
    {
      wait_loop_index--;
    }

    /* Enable ADC */
    LL_ADC_Enable(ADC5);

    /* Poll for ADC ready to convert */
#if (USE_TIMEOUT == 1)
        Timeout = ADC_ENABLE_TIMEOUT_MS;
        #endif /* USE_TIMEOUT */

    while(LL_ADC_IsActiveFlag_ADRDY(ADC5)== 0)
    {
#if (USE_TIMEOUT == 1)
          /* Check Systick counter flag to decrement the time-out value */
          if (LL_SYSTICK_IsActiveCounterFlag())
          {
            if(Timeout-- == 0)
            {
            /* Time-out occurred. Set LED to blinking mode */
            LED_Blinking(LED_BLINK_ERROR);
            }
          }
        #endif /* USE_TIMEOUT */
    }
  }
}

void DMA2_Channel1_ConvCplt_Callback()
{
//  TEST_A_OFF;
  LL_ADC_REG_StopConversion(ADC2);

  for(int i=0; i< ADC_COUNT; i++)
  {
    adc2ValA[i]=__LL_ADC_CALC_DATA_TO_VOLTAGE(adc5Val[0], DMA_ADC2_Buf[i* 2], LL_ADC_RESOLUTION_12B);
    adc2ValB[i]=__LL_ADC_CALC_DATA_TO_VOLTAGE(adc5Val[0], DMA_ADC2_Buf[i* 2+ 1], LL_ADC_RESOLUTION_12B);
  }

  if(CH_LEFT== RUN_MODE)
  {
    if((pmr.mode!= TWIST)|| ((pmr.mode== TWIST)&& phase_2A_flag))
    {
      if(hallA_cnt< HALL_COUNT)
      {
        qsort(adc2ValA, ADC_COUNT, sizeof(uint16_t), compare);
        hallA_low[hallA_cnt]=adc2ValA[0];
        hallA_cnt++;
      }
    }
  }

  if(CH_RIGHT== RUN_MODE)
  {
    if((pmr.mode!= TWIST)|| ((pmr.mode== TWIST)&& phase_2B_flag))
    {
      if(hallB_cnt< HALL_COUNT)
      {
        qsort(adc2ValB, ADC_COUNT, sizeof(uint16_t), compare);
        hallB_low[hallB_cnt]=adc2ValB[0];
        hallB_cnt++;
      }
    }
  }
}

void AdcGrpRegularUnitaryConvComplete_Callback1()
{
  uint16_t adcVal1=LL_ADC_REG_ReadConversionData12(ADC1);

  adc1Val[adc1Ch]=__LL_ADC_CALC_DATA_TO_VOLTAGE(adc5Val[0], adcVal1, LL_ADC_RESOLUTION_12B);

  adc1Ch++;
  if(adc1Ch>= 2)
  {
    adc1Ch=0;
    adc1Done=1;
  }
}

void AdcGrpRegularUnitaryConvComplete_Callback5()
{
  uint16_t adcVal5=LL_ADC_REG_ReadConversionData12(ADC5);

  if(adc5Ch== 0) adc5Val[adc5Ch]=__LL_ADC_CALC_VREFANALOG_VOLTAGE(adcVal5, LL_ADC_RESOLUTION_12B);
  else
  {
    adc5Val[adc5Ch]=__LL_ADC_CALC_TEMPERATURE(adc5Val[0], adcVal5, LL_ADC_RESOLUTION_12B);
  }

  adc5Ch++;
  if(adc5Ch>= 2)
  {
    adc5Ch=0;
    adc5Done=1;
  }
}

void AdcGrpRegularOverrunError_Callback1()
{
  /* Disable ADC group regular overrun interruption */
  LL_ADC_DisableIT_OVR(ADC1);

  /* Error from ADC */
  LOG_PRINT("ADC1 ERROR\r\n");
}

void AdcGrpRegularOverrunError_Callback5()
{
  /* Disable ADC group regular overrun interruption */
  LL_ADC_DisableIT_OVR(ADC5);

  /* Error from ADC */
  LOG_PRINT("ADC5 ERROR\r\n");
}

void ADC2_TransferError_Callback()
{
  /* Error from ADC */
  LOG_PRINT("ADC2 ERROR\r\n");
}
#endif

void QR_Pulse_A_Callback()
{
  if(LL_GPIO_IsInputPinSet(Q_pulse_A_in_GPIO_Port, Q_pulse_A_in_Pin))
  {
    // rising
    READY_A_LOW;

    if(phase_1A_flag)
    {
      phase_1A_count++;
      chA_DA+=chA_uStep;
      if(chA_DA> pmr.intenA) chA_DA=pmr.intenA;

      if(phase_1A_count>= uStep)
      {
        changedFreqA=0;
        phase_1A_count=0;
        phase_1A_flag=0;
        phase_2A_count=0;
        phase_2A_flag=1;
        chA_DA=pmr.intenA;
#if 1
        if((pmr.mode== GRIP)|| (pmr.mode== TAP))
        {
          if(pmr.freq1!= pmr.freq0)
          {
            Q_change_A(pmr.freq1);
            changedFreqA=1;
          }
        }
#endif
        hallA_cnt=0;
      }
      DA_update_A_flag=1;
    }

    if(phase_3A_flag)
    {
      phase_3A_count++;
      chA_DA-=chA_dStep;
      if(chA_DA< CONSTANT_DA) chA_DA=CONSTANT_DA;

      if(phase_3A_count>= dStep)
      {
        phase_3A_count=0;
        phase_3A_flag=0;
        phase_4A_count=0;
        phase_4A_flag=1;
        once_A_flag=1;
        chA_DA=CONSTANT_DA;
      }
      DA_update_A_flag=1;
    }

    if(phase_5A_flag)
    {
      phase_5A_count++;
      chA_DA-=chA_dStep5;
      if(operation_flag== 3)
      {
        if(chA_DA< CONSTANT_DA) chA_DA=CONSTANT_DA;
      }
      else
      {
        if(chA_DA< 0) chA_DA=0;
      }

      if((phase_5A_count>= dStep5)|| ((operation_flag== 3)&& (chA_DA== CONSTANT_DA))|| (chA_DA== 0))
      {
        phase_5A_count=0;
        phase_5A_flag=0;

        if(intenA_Zero)
        {
          intenA_Zero=0;
          RunToPause(CHANNEL_L);
        }
        else
        {
          if(operation_flag== 3) RunToRest(CHANNEL_L);
          else RunToStopA();
        }
      }
      DA_update_A_flag=1;
    }

    if(phase_2A_flag)
    {
      if((LL_ADC_IsEnabled(ADC2)== 1)&& (LL_ADC_IsDisableOngoing(ADC2)== 0)&& (LL_ADC_REG_IsConversionOngoing(ADC2)== 0))
      {
        LL_ADC_REG_StartConversion(ADC2);
//        TEST_A_ON;
      }
    }
  }
  else
  {
    // falling
    READY_A_HIGH;
  }
}

void QR_Pulse_B_Callback()
{
  if(LL_GPIO_IsInputPinSet(Q_pulse_B_in_GPIO_Port, Q_pulse_B_in_Pin))
  {
    // rising
    READY_B_LOW;

    if(phase_1B_flag)
    {
      phase_1B_count++;
      chB_DA+=chB_uStep;
      if(chB_DA> pmr.intenB) chB_DA=pmr.intenB;

      if(phase_1B_count>= uStep)
      {
        changedFreqB=0;
        phase_1B_count=0;
        phase_1B_flag=0;
        phase_2B_count=0;
        phase_2B_flag=1;
        chB_DA=pmr.intenB;
#if 1
        if((pmr.mode== GRIP)|| (pmr.mode== TAP))
        {
          if(pmr.freq1!= pmr.freq0)
          {
            Q_change_B(pmr.freq1);
            changedFreqB=1;
          }
        }
#endif
        hallB_cnt=0;
      }
      DA_update_B_flag=1;
    }

    if(phase_3B_flag)
    {
      phase_3B_count++;
      chB_DA-=chB_dStep;
      if(chB_DA< CONSTANT_DA) chB_DA=CONSTANT_DA;

      if(phase_3B_count>= dStep)
      {
        phase_3B_count=0;
        phase_3B_flag=0;
        phase_4B_count=0;
        phase_4B_flag=1;
        once_B_flag=1;
        chB_DA=CONSTANT_DA;
      }
      DA_update_B_flag=1;
    }

    if(phase_5B_flag)
    {
      phase_5B_count++;
      chB_DA-=chB_dStep5;
      if(operation_flag== 3)
      {
        if(chB_DA< CONSTANT_DA) chB_DA=CONSTANT_DA;
      }
      else
      {
        if(chB_DA< 0) chB_DA=0;
      }

      if((phase_5B_count>= dStep5)|| ((operation_flag== 3)&& (chB_DA== CONSTANT_DA))|| (chB_DA== 0))
      {
        phase_5B_count=0;
        phase_5B_flag=0;

        if(intenB_Zero)
        {
          intenB_Zero=0;
          RunToPause(CHANNEL_R);
        }
        else
        {
          if(operation_flag== 3) RunToRest(CHANNEL_R);
          else RunToStopB();
        }
      }
      DA_update_B_flag=1;
    }

    if(phase_2B_flag)
    {
      if((LL_ADC_IsEnabled(ADC2)== 1)&& (LL_ADC_IsDisableOngoing(ADC2)== 0)&& (LL_ADC_REG_IsConversionOngoing(ADC2)== 0))
      {
        LL_ADC_REG_StartConversion(ADC2);
      }
    }
  }
  else
  {
    // falling
    READY_B_HIGH;
  }
}

void Patient_SW_Callback()
{
//  // stop All
//  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
//  TIM1->CNT=0;
//
//  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
//  TIM8->CNT=0;
//
//  Ready2Standby(CHANNEL_L+ CHANNEL_R);
//
//  SEND_1BYTE(CMD_RUN, 1);

  SEND_1BYTE(CMD_EVENT, 0);
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
int __io_putchar(int ch)
{
#ifdef USE_LPUART
  HAL_UART_Transmit(&hlpuart1, (uint8_t*)&ch, 1, TX_TIMEOUT);
#else
  HAL_UART_Transmit(&huart4, (uint8_t*)&ch, 1, TX_TIMEOUT);
#endif

  return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
#ifdef USE_LPUART
  if(UartHandle->Instance== LPUART1)
  {
    if(LP_RX_Buffer[0]== '0')
    {
      use_debug=1;
      LOG_PRINT("-- Debugging Message ON --\r\n");
    }
    else if(LP_RX_Buffer[0]== '1')
    {
      use_debug=0;
      LOG_PRINT("-- Debugging Message OFF --\r\n");
    }
    else if(LP_RX_Buffer[0]== '2')
    {
      use_debug2=1;
      LOG_PRINT("-- Temperature Message ON --\r\n");
    }
    else if(LP_RX_Buffer[0]== '3')
    {
      use_debug2=0;
      LOG_PRINT("-- Temperature Message OFF --\r\n");
    }
    else if(LP_RX_Buffer[0]== '4')
    {
      use_debug3=1;
      LOG_PRINT("-- HallSensor Message ON --\r\n");
    }
    else if(LP_RX_Buffer[0]== '5')
    {
      use_debug3=0;
      LOG_PRINT("-- HallSensor Message OFF --\r\n");
    }
    else if(LP_RX_Buffer[0]== '6')
    {
      use_debug4=1;
      LOG_PRINT("-- JogDial Message ON --\r\n");
    }
    else if(LP_RX_Buffer[0]== '7')
    {
      use_debug4=0;
      LOG_PRINT("-- JogDial Message OFF --\r\n");
    }

    HAL_UART_Receive_IT(&hlpuart1, LP_RX_Buffer, 1);
  }
#else
  if(UartHandle->Instance== UART4)
  {
    if(RX4_Buffer[0]== '0')
    {
      use_debug=1;
      LOG_PRINT("-- Debugging Message ON --\r\n");
    }
    else if(RX4_Buffer[0]== '1')
    {
      use_debug=0;
      LOG_PRINT("-- Debugging Message OFF --\r\n");
    }
    else if(RX4_Buffer[0]== '2')
    {
      use_debug2=1;
      LOG_PRINT("-- Temperature Message ON --\r\n");
    }
    else if(RX4_Buffer[0]== '3')
    {
      use_debug2=0;
      LOG_PRINT("-- Temperature Message OFF --\r\n");
    }
    else if(RX4_Buffer[0]== '4')
    {
      use_debug3=1;
      LOG_PRINT("-- HallSensor Message ON --\r\n");
    }
    else if(RX4_Buffer[0]== '5')
    {
      use_debug3=0;
      LOG_PRINT("-- HallSensor Message OFF --\r\n");
    }
    else if(RX4_Buffer[0]== '6')
    {
      use_debug4=1;
      LOG_PRINT("-- JogDial Message ON --\r\n");
    }
    else if(RX4_Buffer[0]== '7')
    {
      use_debug4=0;
      LOG_PRINT("-- JogDial Message OFF --\r\n");
    }

    HAL_UART_Receive_IT(&huart4, RX4_Buffer, 1);
  }
#endif
}

/**
 * @brief  Function called in case of error detected in USART IT Handler
 * @param  None
 * @retval None
 */
void USART1_TransferError_Callback()
{
  if(LL_DMA_IsActiveFlag_TE1(DMA1))
  {
    /* Disable DMA1 Rx Channel */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    if(use_debug) DBG_PRINT("USART1 - DMA Rx error\r\n");
  }

  if(LL_DMA_IsActiveFlag_TE2(DMA1))
  {
    /* Disable DMA1 Tx Channel */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    if(use_debug) DBG_PRINT("USART1 - DMA Tx error\r\n");
  }
}

void USART3_TransferError_Callback()
{
  if(LL_DMA_IsActiveFlag_TE3(DMA1))
  {
    /* Disable DMA1 Rx Channel */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    if(use_debug) DBG_PRINT("USART3 - DMA Rx error\r\n");
  }

  if(LL_DMA_IsActiveFlag_TE4(DMA1))
  {
    /* Disable DMA1 Tx Channel */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_4);
    if(use_debug) DBG_PRINT("USART3 - DMA Tx error\r\n");
  }
}

void UART5_TransferError_Callback()
{
  if(LL_DMA_IsActiveFlag_TE5(DMA1))
  {
    /* Disable DMA1 Rx Channel */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_5);
    if(use_debug) DBG_PRINT("UART5 - DMA Rx error\r\n");
  }

  if(LL_DMA_IsActiveFlag_TE6(DMA1))
  {
    /* Disable DMA1 Tx Channel */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_6);
    if(use_debug) DBG_PRINT("UART5 - DMA Tx error\r\n");
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
