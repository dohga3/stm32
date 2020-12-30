//-------------------------------------------------------------------------
// Project    : MS19 Project (MainSystem Board)
//-------------------------------------------------------------------------
// MCU        : STM32G474QETx
// CLOCK      : 170MHz
// Date       : 2020.02.21
// Comfiler   : gcc (TBD)
// File Name  : Device.c
// made by    : dohga
//-------------------------------------------------------------------------

#include "main.h"

//-------------------------------------------------------------------------
// Power DA Function
//-------------------------------------------------------------------------
void POWER_DA_CONTROL_A(uint32_t data)
{
  uint8_t d4922_data;
  uint8_t d4922_command;

  LL_GPIO_ResetOutputPin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin);

  d4922_command=(data>> 8)& 0xff;
  d4922_command=0x70| d4922_command;
  d4922_data=data& 0xff;

  LL_SPI_TransmitData8(SPI1, d4922_command);
  while(LL_SPI_IsActiveFlag_BSY(SPI1))
  {
  }

  LL_SPI_TransmitData8(SPI1, d4922_data);
  while(LL_SPI_IsActiveFlag_BSY(SPI1))
  {
  }

  LL_GPIO_SetOutputPin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin);
}

void POWER_DA_CONTROL_B(uint32_t data)
{
  uint8_t d4922_data;
  uint8_t d4922_command;

  LL_GPIO_ResetOutputPin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin);

  d4922_command=(data>> 8)& 0xff;
  d4922_command=0xf0| d4922_command;
  d4922_data=data& 0xff;

  LL_SPI_TransmitData8(SPI1, d4922_command);
  while(LL_SPI_IsActiveFlag_BSY(SPI1))
  {
  }

  LL_SPI_TransmitData8(SPI1, d4922_data);
  while(LL_SPI_IsActiveFlag_BSY(SPI1))
  {
  }

  LL_GPIO_SetOutputPin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin);
}

/* -----------------------------------------------------------------------
 To get TIM1 counter clock at 85 MHz, the prescaler is computed as follows:
 Prescaler = (TIM1CLK / TIM1 counter clock) - 1
 Prescaler = ((SystemCoreClock) /85 MHz) - 1

 To get TIM1 output clock at 85 KHz, the period (ARR)) is computed as follows:
 ARR = (TIM1 counter clock / TIM1 output clock) - 1
 = 999

 TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR + 1)* 100 = 50%
 TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR + 1)* 100 = 37.5%
 TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR + 1)* 100 = 25%
 TIM1 Channel4 duty cycle = (TIM1_CCR4/ TIM1_ARR + 1)* 100 = 12.5%
 ----------------------------------------------------------------------- */
//-------------------------------------------------------------------------
// Q_change Function
//-------------------------------------------------------------------------
void Q_change_A(uint32_t Q_FREQ_A)
{
  uint32_t PRESCALER_VALUE=0;
  uint32_t PERIOD_VALUE=0, PULSE_VALUE=0;

  if((Q_FREQ_A>= 1)&& (Q_FREQ_A<= 150))
  {
    // timer set
    PRESCALER_VALUE=(SystemCoreClock/ (TIM_COUNTER_CONST* Q_FREQ_A))- 1;
    TIM1->PSC=PRESCALER_VALUE;

    PERIOD_VALUE=TIM_COUNTER_CONST- 1;
    TIM1->ARR=PERIOD_VALUE;

    PULSE_VALUE=(PERIOD_VALUE+ 1)* 2/ (1000/ Q_FREQ_A);
    TIM1->CCR1=PULSE_VALUE;

    TIM1->CNT=0;
  }
}

void Ready_A_Start(void)
{
  /* Enable ExtiLine (Rising/Falling Edge) */
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
}

void Ready_A_Stop(void)
{
  /* Disable ExtiLine */
  LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_0);

  READY_A_HIGH;
}

void Q_change_B(uint32_t Q_FREQ_B)
{
  uint32_t PRESCALER_VALUE=0;
  uint32_t PERIOD_VALUE=0, PULSE_VALUE=0;

  if((Q_FREQ_B>= 1)&& (Q_FREQ_B<= 150))
  {
    // timer set
    PRESCALER_VALUE=(SystemCoreClock/ (TIM_COUNTER_CONST* Q_FREQ_B))- 1;
    TIM8->PSC=PRESCALER_VALUE;

    PERIOD_VALUE=TIM_COUNTER_CONST- 1;
    TIM8->ARR=PERIOD_VALUE;

    PULSE_VALUE=(PERIOD_VALUE+ 1)* 2/ (1000/ Q_FREQ_B);
    TIM8->CCR1=PULSE_VALUE;

    TIM8->CNT=0;
  }
}

void Ready_B_Start(void)
{
  /* Enable ExtiLine (Rising/Falling Edge) */
  LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
}

void Ready_B_Stop(void)
{
  /* Disable ExtiLine */
  LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_1);

  READY_B_HIGH;
}

/* End of Code */
