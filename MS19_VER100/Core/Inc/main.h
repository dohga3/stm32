/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
//#include <math.h>

#include "DS2482.h"
#include "aes.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void QR_Pulse_A_Callback(void);
void QR_Pulse_B_Callback(void);
void Patient_SW_Callback(void);
//void Door_Interlock_Callback(uint16_t GPIO_Pin);

void DMA1_Channel1_ReceiveCplt_Callback(void);
void DMA1_Channel2_TransmitCplt_Callback(void);
void USART1_TransferError_Callback(void);

void DMA1_Channel3_ReceiveCplt_Callback(void);
void DMA1_Channel4_TransmitCplt_Callback(void);
void USART3_TransferError_Callback(void);

void DMA1_Channel5_ReceiveCplt_Callback(void);
void DMA1_Channel6_TransmitCplt_Callback(void);
void UART5_TransferError_Callback(void);

void AdcGrpRegularUnitaryConvComplete_Callback1(void);
void AdcGrpRegularOverrunError_Callback1(void);

void AdcGrpRegularUnitaryConvComplete_Callback5(void);
void AdcGrpRegularOverrunError_Callback5(void);

//void DMA2_Channel1_ConvHalfCplt_Callback(void);
void DMA2_Channel1_ConvCplt_Callback(void);
void ADC2_TransferError_Callback(void);

void AP_A_FAN_ON(void);
void AP_A_FAN_OFF(void);
void AP_B_FAN_ON(void);
void AP_B_FAN_OFF(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CONTACTER_CTL_Pin LL_GPIO_PIN_13
#define CONTACTER_CTL_GPIO_Port GPIOC
#define AP_A_HALL_Pin LL_GPIO_PIN_0
#define AP_A_HALL_GPIO_Port GPIOC
#define AP_B_HALL_Pin LL_GPIO_PIN_1
#define AP_B_HALL_GPIO_Port GPIOC
#define Pwr_Btn_LED_Pin LL_GPIO_PIN_2
#define Pwr_Btn_LED_GPIO_Port GPIOC
#define Door_interlock_R_Pin LL_GPIO_PIN_3
#define Door_interlock_R_GPIO_Port GPIOC
#define LD3_Pin LL_GPIO_PIN_2
#define LD3_GPIO_Port GPIOF
#define AP_A_TEMP_Pin LL_GPIO_PIN_0
#define AP_A_TEMP_GPIO_Port GPIOA
#define AP_B_TEMP_Pin LL_GPIO_PIN_1
#define AP_B_TEMP_GPIO_Port GPIOA
#define Rsrv1_tx_Pin LL_GPIO_PIN_2
#define Rsrv1_tx_GPIO_Port GPIOA
#define Rsrv1_rx_Pin LL_GPIO_PIN_3
#define Rsrv1_rx_GPIO_Port GPIOA
#define AP_A_FAN_ctl_Pin LL_GPIO_PIN_4
#define AP_A_FAN_ctl_GPIO_Port GPIOA
#define Jog_encoder_A_Pin LL_GPIO_PIN_6
#define Jog_encoder_A_GPIO_Port GPIOA
#define Door_interlock_L_Pin LL_GPIO_PIN_4
#define Door_interlock_L_GPIO_Port GPIOC
#define Front_LED_OnOff_Pin LL_GPIO_PIN_5
#define Front_LED_OnOff_GPIO_Port GPIOC
#define Q_pulse_A_in_Pin LL_GPIO_PIN_0
#define Q_pulse_A_in_GPIO_Port GPIOB
#define Q_pulse_A_in_EXTI_IRQn EXTI0_IRQn
#define Q_pulse_B_in_Pin LL_GPIO_PIN_1
#define Q_pulse_B_in_GPIO_Port GPIOB
#define Q_pulse_B_in_EXTI_IRQn EXTI1_IRQn
#define Patient_interlock_Pin LL_GPIO_PIN_2
#define Patient_interlock_GPIO_Port GPIOB
#define Patient_interlock_EXTI_IRQn EXTI2_IRQn
#define reserve1_temp_Pin LL_GPIO_PIN_7
#define reserve1_temp_GPIO_Port GPIOE
#define reserve2_temp_Pin LL_GPIO_PIN_8
#define reserve2_temp_GPIO_Port GPIOE
#define Q_PULSE_A_Pin LL_GPIO_PIN_9
#define Q_PULSE_A_GPIO_Port GPIOE
#define READY_A_Pin LL_GPIO_PIN_10
#define READY_A_GPIO_Port GPIOE
#define SYS_FAN_sense_Pin LL_GPIO_PIN_11
#define SYS_FAN_sense_GPIO_Port GPIOE
#define SYS_FAN_CTL_Pin LL_GPIO_PIN_15
#define SYS_FAN_CTL_GPIO_Port GPIOE
#define Debug_RX_Pin LL_GPIO_PIN_10
#define Debug_RX_GPIO_Port GPIOB
#define Debug_TX_Pin LL_GPIO_PIN_11
#define Debug_TX_GPIO_Port GPIOB
#define KEY_CHECK_Pin LL_GPIO_PIN_12
#define KEY_CHECK_GPIO_Port GPIOB
#define PWR1_TX_Pin LL_GPIO_PIN_8
#define PWR1_TX_GPIO_Port GPIOD
#define PWR1_RX_Pin LL_GPIO_PIN_9
#define PWR1_RX_GPIO_Port GPIOD
#define AP_A_CONNECT_Pin LL_GPIO_PIN_10
#define AP_A_CONNECT_GPIO_Port GPIOD
#define AP_B_CONNECT_Pin LL_GPIO_PIN_11
#define AP_B_CONNECT_GPIO_Port GPIOD
#define LD2_Pin LL_GPIO_PIN_12
#define LD2_GPIO_Port GPIOD
#define AP_B_LED1_Pin LL_GPIO_PIN_13
#define AP_B_LED1_GPIO_Port GPIOD
#define AP_B_LED2_Pin LL_GPIO_PIN_14
#define AP_B_LED2_GPIO_Port GPIOD
#define AP_B_LED3_Pin LL_GPIO_PIN_15
#define AP_B_LED3_GPIO_Port GPIOD
#define Q_PULSE_B_Pin LL_GPIO_PIN_6
#define Q_PULSE_B_GPIO_Port GPIOC
#define READY_B_Pin LL_GPIO_PIN_7
#define READY_B_GPIO_Port GPIOC
#define Jog_encoder_B_Pin LL_GPIO_PIN_8
#define Jog_encoder_B_GPIO_Port GPIOA
#define GUI_TX_Pin LL_GPIO_PIN_9
#define GUI_TX_GPIO_Port GPIOA
#define GUI_RX_Pin LL_GPIO_PIN_10
#define GUI_RX_GPIO_Port GPIOA
#define AP_B_FAN_ctl_Pin LL_GPIO_PIN_11
#define AP_B_FAN_ctl_GPIO_Port GPIOA
#define Jog_PushButton_Pin LL_GPIO_PIN_12
#define Jog_PushButton_GPIO_Port GPIOA
#define LD4_Pin LL_GPIO_PIN_6
#define LD4_GPIO_Port GPIOF
#define LD1_Pin LL_GPIO_PIN_15
#define LD1_GPIO_Port GPIOA
#define Rsrv2_tx_Pin LL_GPIO_PIN_10
#define Rsrv2_tx_GPIO_Port GPIOC
#define Rsrv2_rx_Pin LL_GPIO_PIN_11
#define Rsrv2_rx_GPIO_Port GPIOC
#define PWR2_TX_Pin LL_GPIO_PIN_12
#define PWR2_TX_GPIO_Port GPIOC
#define AC_STATE_CHECK_Pin LL_GPIO_PIN_0
#define AC_STATE_CHECK_GPIO_Port GPIOD
#define PC_brd_SW_Pin LL_GPIO_PIN_1
#define PC_brd_SW_GPIO_Port GPIOD
#define PWR2_RX_Pin LL_GPIO_PIN_2
#define PWR2_RX_GPIO_Port GPIOD
#define Front_LED_PWM_Pin LL_GPIO_PIN_3
#define Front_LED_PWM_GPIO_Port GPIOD
#define Jog_BackLight_Pin LL_GPIO_PIN_4
#define Jog_BackLight_GPIO_Port GPIOD
#define AP_A_LED1_Pin LL_GPIO_PIN_5
#define AP_A_LED1_GPIO_Port GPIOD
#define AP_A_LED2_Pin LL_GPIO_PIN_6
#define AP_A_LED2_GPIO_Port GPIOD
#define AP_A_LED3_Pin LL_GPIO_PIN_7
#define AP_A_LED3_GPIO_Port GPIOD
#define BUZZER_Pin LL_GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOB
#define MCP4922_CS_Pin LL_GPIO_PIN_6
#define MCP4922_CS_GPIO_Port GPIOB
#define REMOTE_DATA_Pin LL_GPIO_PIN_7
#define REMOTE_DATA_GPIO_Port GPIOB
#define REMOTE_CTL_Pin LL_GPIO_PIN_9
#define REMOTE_CTL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define SystemVersion     100     // ex) V00.01 *100 => 1

#define TIM_COUNTER_CLOCK  (uint32_t)1000000
#define TIM_COUNTER_CONST   60000

#define CONSTANT_DA     1350
#define ADC_COUNT       125
#define HALL_COUNT      200

#define WATERMARK       0xE0  // alpha

//----------------------------------------
// GUI CMD Define
//----------------------------------------
#define CMD_ERROR         'E'
#define CMD_HALL          'H'
#define CMD_JOG           'J'
#define CMD_MEMORY        'M'
#define CMD_EVENT         'N'
#define CMD_GUI_ONOFF     'O'
#define CMD_PARAMETER     'P'
#define CMD_RUN           'R'
#define CMD_STATUS        'S'
#define CMD_TEMPER        'T'
#define CMD_UPDATE        'U'
#define CMD_VERSION       'V'
#define CMD_undefine      'Z'

//#define cmd_reaction1     'a'
//#define cmd_reaction2     'b'

//----------------------------------------
// Power CMD Define
//----------------------------------------
#define p_Version    'V'
#define p_Error      'E'
#define p_Status     'S'

//------------------------------------------------------------------------------
// Sensor Check Time define
//------------------------------------------------------------------------------
#define ChkTime_10ms        10
#define ChkTime_20ms        20
#define ChkTime_50ms        50
#define ChkTime_100ms       100
#define ChkTime_200ms       200
#define ChkTime_400ms       400
#define ChkTime_500ms       500
//#define ChkTime_1000ms      1000
//#define ChkTime_1500ms      1500
//#define ChkTime_2500ms      2500

#define ChkTime_1sec        1000
#define ChkTime_2sec        2000
//#define ChkTime_5sec        5000
//#define ChkTime_10sec       10000
#define ChkTime_20sec       20000
#define ChkTime_30sec       30000
#define ChkTime_150sec      150000

//------------------------------------------------------------------------------
// ASCII CODE DEFINITION
//------------------------------------------------------------------------------
#define ACK       0x06
#define NAK       0x15
#define DLE       0x10
#define STX       0x02
#define ETX       0x03

// Serial 1
#define rx_buff_size1      30
#define tx_buff_size1      30
#define temp_buff_size1    30

uint8_t _bit_txnak1;
uint8_t _bit_rxnak1;
uint8_t buflen1;
uint8_t bccchk1;
uint8_t GUI_CMD;

//uint8_t GUI_Response;
uint8_t GUI_Boot_OK;

__IO uint8_t DMA_USART1_buf[1], DMA_USART3_buf[1], DMA_UART5_buf[1];

uint8_t rbuf1[rx_buff_size1];
uint8_t rbuf_temp1[1];
uint8_t tbuf1[tx_buff_size1];
uint8_t tempbuf1[temp_buff_size1];

uint8_t rxlen1, txlen1, tx_rp1;
uint8_t bcc_check_buffer1;

// Serial 3
#define rx_buff_size3      20
#define tx_buff_size3      20
#define temp_buff_size3    20

uint8_t _bit_txnak3;
uint8_t _bit_rxnak3;
uint8_t buflen3;
uint8_t bccchk3;
uint8_t POWER_CMD3;

uint8_t rbuf3[rx_buff_size3];
uint8_t rbuf_temp3[1];
uint8_t tbuf3[tx_buff_size3];
uint8_t tempbuf3[temp_buff_size3];

uint8_t rxlen3, txlen3, tx_rp3;
uint8_t bcc_check_buffer3;

// Serial 5
#define rx_buff_size5      20
#define tx_buff_size5      20
#define temp_buff_size5    20

uint8_t _bit_txnak5;
uint8_t _bit_rxnak5;
uint8_t buflen5;
uint8_t bccchk5;
uint8_t POWER_CMD5;

uint8_t rbuf5[rx_buff_size5];
uint8_t rbuf_temp5[1];
uint8_t tbuf5[tx_buff_size5];
uint8_t tempbuf5[temp_buff_size5];

uint8_t rxlen5, txlen5, tx_rp5;
uint8_t bcc_check_buffer5;

uint32_t PSC_VerCheck1, PSC_VerCheck2;
uint32_t PSC_Version1, PSC_Version2;
uint32_t AC_VoltBuffer1, AC_VoltBuffer2;
uint8_t ver_count;

//uint8_t temp_mon;
uint8_t use_debug, use_debug2, use_debug3, use_debug4;

//uint32_t count_100_ticks;

uint8_t veryfirst_boot;
uint8_t operation_flag;

// channel A
uint8_t phase_1A_flag;
__IO uint32_t phase_1A_count;
uint8_t phase_2A_flag;
__IO uint32_t phase_2A_count;
uint8_t phase_3A_flag;
__IO uint32_t phase_3A_count;
uint8_t phase_4A_flag;
__IO uint32_t phase_4A_count;
uint8_t once_A_flag;
//__IO uint32_t chA_step;
uint32_t chA_uStep, chA_dStep;
//uint32_t chA_uStep_new, chA_dStep_new;
signed int chA_DA;
__IO uint8_t DA_update_A_flag;
uint8_t reaction_A_flag;
__IO uint8_t changedFreqA;

uint32_t uStep, dStep;
//uint8_t intensity_ch;

uint32_t dStep5, chA_dStep5, chB_dStep5;
uint8_t phase_5A_flag, phase_5B_flag;
__IO uint32_t phase_5A_count, phase_5B_count;

__IO uint8_t _next_step_;
__IO uint8_t _twist_once_;
__IO uint8_t _master_flag;
__IO uint8_t pending_flag;

// channel B
uint8_t phase_1B_flag;
__IO uint32_t phase_1B_count;
uint8_t phase_2B_flag;
__IO uint32_t phase_2B_count;
uint8_t phase_3B_flag;
__IO uint32_t phase_3B_count;
uint8_t phase_4B_flag;
__IO uint32_t phase_4B_count;
uint8_t once_B_flag;
//__IO uint32_t chB_step;
uint32_t chB_uStep, chB_dStep;
//uint32_t chB_uStep_new, chB_dStep_new;
signed int chB_DA;
__IO uint8_t DA_update_B_flag;
uint8_t reaction_B_flag;
__IO uint8_t changedFreqB;

uint32_t lastEncoded;
uint8_t Encoder_SW_flag;
signed int encoderValue;

uint8_t encoder_flag;
uint32_t encoder_count;

uint8_t power_on_flag;
uint32_t power_on_count;
uint8_t power_off_flag;
uint32_t power_off_count;

uint8_t memRead_flag, memRead_cnt;
//uint8_t write_fail_flag;
//uint8_t copy_fail_flag;
//uint8_t erase_fail_flag;

//uint32_t old_intenA;
//uint32_t old_intenB;

uint8_t waitingACK, waitingACK_done, waitingACK_timeover, pwr_btn_pressed;
uint32_t waitingACK_count;

__IO uint8_t CH_LEFT, old_CH_L;
__IO uint8_t CH_RIGHT, old_CH_R;

uint32_t AP_A_Temp, AP_B_Temp;
uint32_t AP_A_Fan_Active, AP_B_Fan_Active;

uint8_t intenA_Zero, intenB_Zero;
__IO uint8_t dischargeA, dischargeB;
uint8_t intenA_reply_cnt, intenB_reply_cnt;
uint8_t chA_DA_zero, chB_DA_zero;

/* for test */
uint8_t hb_flag;
uint32_t hb_cnt;
//uint8_t test_num, memory_flag;
//uint32_t memory_count;
//uint8_t adc2Start;
//uint8_t adc5Start;
//uint32_t sumLowA, sumLowB;
//uint16_t sumA_cnt, sumB_cnt;
#if 0
uint8_t front_LED_flag, freq_plus, front_LED_freq, freq_change;
uint32_t front_LED_count;
uint8_t stay_flag;
uint32_t stay_count;
#endif

uint8_t countTemp, check15sec, commErr;

struct STATUS_FLAG1
{
  union
  {
    struct
    {
      uint8_t bStatus11 :1; //AP A disonnect
      uint8_t bStatus12 :1; //AP B disonnect
      uint8_t bStatus13 :1;
      uint8_t bStatus14 :1;
      uint8_t bStatus15 :1;
      uint8_t bStatus16 :1;
      uint8_t bStatus17 :1;
      uint8_t bStatus18 :1;
    };
    uint8_t STATUS1B;
  };
};
struct STATUS_FLAG1 SF1;
uint8_t oldSTATUS1B;

struct STATUS_FLAG2
{
  union
  {
    struct
    {
      uint8_t bStatus21 :1; // AP A Temp.
      uint8_t bStatus22 :1; // AP B Temp.
      uint8_t bStatus23 :1;
      uint8_t bStatus24 :1;
      uint8_t bStatus25 :1;
      uint8_t bStatus26 :1;
      uint8_t bStatus27 :1;
      uint8_t bStatus28 :1;
    };
    uint8_t STATUS2B;
  };
};
struct STATUS_FLAG2 SF2;
uint8_t oldSTATUS2B;

struct STATUS_FLAG3
{
  union
  {
    struct
    {
      uint8_t bStatus31 :1;
      uint8_t bStatus32 :1;
      uint8_t bStatus33 :1;
      uint8_t bStatus34 :1;
      uint8_t bStatus35 :1;
      uint8_t bStatus36 :1;
      uint8_t bStatus37 :1;
      uint8_t bStatus38 :1;
    };
    uint8_t STATUS3B;
  };
};
struct STATUS_FLAG3 SF3;
uint8_t oldSTATUS3B;

struct STATUS_FLAG4
{
  union
  {
    struct
    {
      uint8_t bStatus41 :1;
      uint8_t bStatus42 :1;
      uint8_t bStatus43 :1;
      uint8_t bStatus44 :1;
      uint8_t bStatus45 :1;
      uint8_t bStatus46 :1;
      uint8_t bStatus47 :1;
      uint8_t bStatus48 :1;
    };
    uint8_t STATUS4B;
  };
};
struct STATUS_FLAG4 SF4;
uint8_t oldSTATUS4B;
uint8_t sendStatus;

struct ERROR_FLAG1
{
  union
  {
    struct
    {
      uint8_t bError11 :1;  //OVP PSC1
      uint8_t bError12 :1;  //OVP PSC2
      uint8_t bError13 :1;  //OTP PSC1
      uint8_t bError14 :1;  //OTP PSC2
      uint8_t bError15 :1;  //OCP PSC1
      uint8_t bError16 :1;  //OCP PSC2
      uint8_t bError17 :1;  //comm. PSC1
      uint8_t bError18 :1;  //comm. PSC2
    };
    uint8_t ERROR1B;
  };
};
struct ERROR_FLAG1 EF1;
uint8_t oldERROR1B;

struct ERROR_FLAG2
{
  union
  {
    struct
    {
      uint8_t bError21 :1;  //discharge relay PSC1
      uint8_t bError22 :1;  //discharge relay PSC2
      uint8_t bError23 :1;
      uint8_t bError24 :1;
      uint8_t bError25 :1;
      uint8_t bError26 :1;  //door
      uint8_t bError27 :1;  //contactor
      uint8_t bError28 :1;  //MCU temp. sensor
    };
    uint8_t ERROR2B;
  };
};
struct ERROR_FLAG2 EF2;
uint8_t oldERROR2B;

struct ERROR_FLAG3
{
  union
  {
    struct
    {
      uint8_t bError31 :1;  //AP A disonnect
      uint8_t bError32 :1;  //AP B disonnect
      uint8_t bError33 :1;  //AP A out H
      uint8_t bError34 :1;  //AP B out H
      uint8_t bError35 :1;  //AP A out L
      uint8_t bError36 :1;  //AP B out L
      uint8_t bError37 :1;
      uint8_t bError38 :1;
    };
    uint8_t ERROR3B;
  };
};
struct ERROR_FLAG3 EF3;
uint8_t oldERROR3B;

struct ERROR_FLAG4
{
  union
  {
    struct
    {
      uint8_t bError41 :1;  //AP memory fail 1
      uint8_t bError42 :1;  //AP memory fail 2
      uint8_t bError43 :1;  //AP memory data anomaly 0
      uint8_t bError44 :1;  //AP memory data anomaly 1
      uint8_t bError45 :1;
      uint8_t bError46 :1;
      uint8_t bError47 :1;
      uint8_t bError48 :1;
    };
    uint8_t ERROR4B;
  };
};
struct ERROR_FLAG4 EF4;
uint8_t oldERROR4B;
uint8_t sendError;

void Pwr_Btn_Check(void);
void System_Off(void);

// USART1
void SEND_1BYTE(uint8_t name, uint8_t _byte);
#ifdef USE_1WIRE_MEMORY
void SEND_MEMREPLY(uint8_t D0);
void SEND_MEMREAD(uint8_t *readVal);
#endif
void SEND_TEMP(uint16_t D01, uint16_t D23, uint8_t D4);
void SEND_HALL(uint8_t D0, uint16_t D12, uint16_t D34);
void SEND_VERSION(void);
void SEND_STATUS(void);
void SEND_ERROR(void);
void CHK_CMD(void);
//void SEND_TEMPERATURE(uint32_t w_temp, uint32_t k_temp);
//void SEND_COUNT(uint8_t ch);
//void SEND_PowerStatus(void);

// USART3
void _send_cmd3(uint8_t name);
void _send_1byte3(uint8_t name, uint8_t _byte);
//unsigned _IGBT_temperature3(uint32_t temp);
void _chk_cmd3(void);

// UART5
void _send_cmd5(uint8_t name);
void _send_1byte5(uint8_t name, uint8_t _byte);
//unsigned _IGBT_temperature5(uint32_t temp);
void _chk_cmd5(void);

typedef struct
{
  uint8_t PCount;
//  uint8_t PSC1;
//  uint8_t PSC2;
//  uint8_t Check_PSC1;
//  uint8_t Check_PSC2;
//  uint8_t Fail_PSC1;
//  uint8_t Fail_PSC2;
  uint8_t reply;
} UART;
UART _uart;

typedef struct
{
  uint8_t loc;
  uint8_t ch;
  uint8_t id;
  uint8_t len;
  uint8_t asc[10];
} I2CMEM;
I2CMEM _memory;
uint8_t mem_done;

typedef struct
{
//  uint8_t freqA;
//  uint8_t freqB;
//  uint8_t uTimeA;
//  uint8_t uTimeA_new;
//  uint8_t uTimeB;
//  uint8_t uTimeB_new;
//  uint8_t sTimeA;
//  uint8_t sTimeA_new;
//  uint8_t sTimeB;
//  uint8_t sTimeB_new;
//  uint8_t dTimeA;
//  uint8_t dTimeA_new;
//  uint8_t dTimeB;
//  uint8_t dTimeB_new;
//  uint8_t offTimeA;
//  uint8_t offTimeB;
  uint8_t freq0;
  uint8_t freq1;
  uint8_t freq2;
  uint8_t freq3;
  uint8_t freq4;
  uint8_t freq5;
  uint8_t uTime;
  uint8_t s1;
  uint8_t s2;
  uint8_t s3;
  uint8_t s4;
  uint8_t s5;
  uint8_t sTime;
  uint8_t dTime;
  uint8_t oTime;

  uint8_t intenCh;
  uint32_t intenA;
  uint32_t intenA_new;
  uint32_t intenB;
  uint32_t intenB_new;
  uint8_t mode;
  uint8_t linked;
//  uint8_t enableA;
//  uint8_t enableB;
} PARAMETER;
PARAMETER pmr;

//------------------------------------------------------------------------------
// OTHER DEFINITION
//------------------------------------------------------------------------------
void Q_change_A(uint32_t Q_FREQ_A);
void Ready_A_Start(void);
void Ready_A_Stop(void);

void Q_change_B(uint32_t Q_FREQ_B);
void Ready_B_Start(void);
void Ready_B_Stop(void);

typedef struct
{
//  uint32_t _bPatientSW_CNT;
  uint32_t _bDoorOpenR_CNT;
  uint32_t _bDoorOpenL_CNT;
  uint32_t _bJogPushSW_CNT;
  uint32_t _bContactor_CNT;
  uint32_t _bPwrButton_CNT;
  uint8_t _OldModeA;
  uint8_t _OldModeB;
//  uint8_t _bPowerStatus;
//    uint8_t _bAutoResponse;
  uint32_t _bCheckDelay;
  uint32_t _BuzzerCNT;
  uint32_t _BuzzerTIME;
//  uint32_t bSys_fan_CNT;
  uint32_t bAP_A_conn_CNT;
  uint32_t bAP_B_conn_CNT;
} IO_STATUS;
IO_STATUS _io_CHK;

//void Status_Check(void);
void Error_Check(void);
//void Buzzer_sound(uint8_t t);
//void Buzzer_long(uint8_t t);

//uint32_t timer_getticks(void);
//uint32_t timer_elapse(uint32_t initial);

enum
{
  STOP_MODE=0, // 0
    PAUSE_MODE,   // 1
    RUN_MODE,     // 2
    CHANGE_MODE,  // 3
    ERROR_MODE,   // 4
    REST_MODE     // 5
};

enum
{
  PRESSED=0, ROTATE_CW, ROTATE_CCW
};

enum
{
  REST=0,   // 0
    TWIST,  // 1
    HOLD,   // 2
    GRIP,   // 3
    TAP,    // 4
    SETUP   // 5
};

enum
{
  CHANNEL_L=1, CHANNEL_R
};

enum
{
  PATIENT_INTERLOCK=0,  // 0
    DOOR_INTERLOCK_R,   // 1
    DOOR_INTERLOCK_L,   // 2
    AP_A_CONNECT,       // 3
    AP_B_CONNECT,       // 4
    AP_A_HALL_HIGH,     // 5
    AP_A_HALL_LOW,      // 6
    AP_B_HALL_HIGH,     // 7
    AP_B_HALL_LOW,      // 8
    AP_A_TEMPERATURE,   // 9
    AP_B_TEMPERATURE,   // 10
    SYS_FAN,            // 11
    CONTACTOR           // 12
};

void system_init(void);
void GPIO_direction(void);
//void Error_Reset(void);
//void Auto_Response(void);

void POWER_DA_CONTROL_A(uint32_t data);
void POWER_DA_CONTROL_B(uint32_t data);

void RunToStopA(void);
void RunToStopB(void);
void RunToStopVal(void);
void RunToPause(uint8_t ch);
void RunToRest(uint8_t ch);

uint32_t byte2int(uint8_t high, uint8_t low);

//#define UART_PRINT(...)     printf(__VA_ARGS__)
#define DBG_PRINT(...)        printf(__VA_ARGS__)

#define LOG_PRINT(...)        printf(__VA_ARGS__);  \
                                fflush(stdout);

#define delay_ms(x)     LL_mDelay(x)  //HAL_Delay(x)
#define TX_TIMEOUT      1

#if 1
#define REMOTE_ON       LL_GPIO_SetOutputPin(REMOTE_CTL_GPIO_Port, REMOTE_CTL_Pin)
#define REMOTE_OFF      LL_GPIO_ResetOutputPin(REMOTE_CTL_GPIO_Port, REMOTE_CTL_Pin)
#else
#define U373_LE_HIGH    LL_GPIO_SetOutputPin(REMOTE_CTL_GPIO_Port, REMOTE_CTL_Pin)
#define U373_LE_LOW     LL_GPIO_ResetOutputPin(REMOTE_CTL_GPIO_Port, REMOTE_CTL_Pin)

#define U373_D_HIGH     LL_GPIO_SetOutputPin(REMOTE_DATA_GPIO_Port, REMOTE_DATA_Pin)
#define U373_D_LOW      LL_GPIO_ResetOutputPin(REMOTE_DATA_GPIO_Port, REMOTE_DATA_Pin)
#endif

#define PUSH_LED_ON     LL_GPIO_SetOutputPin(Pwr_Btn_LED_GPIO_Port, Pwr_Btn_LED_Pin)
#define PUSH_LED_OFF    LL_GPIO_ResetOutputPin(Pwr_Btn_LED_GPIO_Port, Pwr_Btn_LED_Pin)

#define CONTACTOR_ON    LL_GPIO_SetOutputPin(CONTACTER_CTL_GPIO_Port, CONTACTER_CTL_Pin)
#define CONTACTOR_OFF   LL_GPIO_ResetOutputPin(CONTACTER_CTL_GPIO_Port, CONTACTER_CTL_Pin)

#define PC_SW_ON        LL_GPIO_SetOutputPin(PC_brd_SW_GPIO_Port, PC_brd_SW_Pin)
#define PC_SW_OFF       LL_GPIO_ResetOutputPin(PC_brd_SW_GPIO_Port, PC_brd_SW_Pin)

#define BUZZER_ON       LL_GPIO_SetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin)
#define BUZZER_OFF      LL_GPIO_ResetOutputPin(BUZZER_GPIO_Port, BUZZER_Pin)

#define READY_A_HIGH    LL_GPIO_SetOutputPin(READY_A_GPIO_Port, READY_A_Pin)
#define READY_A_LOW     LL_GPIO_ResetOutputPin(READY_A_GPIO_Port, READY_A_Pin)
#define READY_B_HIGH    LL_GPIO_SetOutputPin(READY_B_GPIO_Port, READY_B_Pin)
#define READY_B_LOW     LL_GPIO_ResetOutputPin(READY_B_GPIO_Port, READY_B_Pin)

#define AP_A_FAN_START    LL_GPIO_SetOutputPin(AP_A_FAN_ctl_GPIO_Port, AP_A_FAN_ctl_Pin)
#define AP_A_FAN_STOP     LL_GPIO_ResetOutputPin(AP_A_FAN_ctl_GPIO_Port, AP_A_FAN_ctl_Pin)

#define AP_B_FAN_START    LL_GPIO_SetOutputPin(AP_B_FAN_ctl_GPIO_Port, AP_B_FAN_ctl_Pin)
#define AP_B_FAN_STOP     LL_GPIO_ResetOutputPin(AP_B_FAN_ctl_GPIO_Port, AP_B_FAN_ctl_Pin)

#define FRONT_LED_ON    LL_GPIO_SetOutputPin(Front_LED_OnOff_GPIO_Port, Front_LED_OnOff_Pin)
#define FRONT_LED_OFF   LL_GPIO_ResetOutputPin(Front_LED_OnOff_GPIO_Port, Front_LED_OnOff_Pin)

#define LD2_ON    LL_GPIO_SetOutputPin(GPIOD, LL_GPIO_PIN_12)
#define LD2_OFF   LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_12)

#define TEST_A_ON    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3)
#define TEST_A_OFF   LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3)
//#define TEST_B_ON    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4)
//#define TEST_B_OFF   LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
