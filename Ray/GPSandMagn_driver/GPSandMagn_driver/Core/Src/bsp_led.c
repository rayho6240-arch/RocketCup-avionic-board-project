#include "bsp_led.h"

static void LED_Delay(uint16_t delay_ms);
static void LED_BusyDelay(uint16_t delay_ms);

void LED_On(void)
{
  HAL_GPIO_WritePin(LED_debug_GPIO_Port, LED_debug_Pin, GPIO_PIN_SET);
}

void LED_Off(void)
{
  HAL_GPIO_WritePin(LED_debug_GPIO_Port, LED_debug_Pin, GPIO_PIN_RESET);
}

void LED_Toggle(void)
{
  HAL_GPIO_TogglePin(LED_debug_GPIO_Port, LED_debug_Pin);
}

void LED_Blink(uint8_t times, uint16_t delay_ms)
{
  for (uint8_t index = 0U; index < times; index++)
  {
    LED_On();
    LED_Delay(delay_ms);
    LED_Off();
    LED_Delay(delay_ms);
  }
}

static void LED_Delay(uint16_t delay_ms)
{
  if (__get_IPSR() == 0U)
  {
    HAL_Delay(delay_ms);
  }
  else
  {
    LED_BusyDelay(delay_ms);
  }
}

static void LED_BusyDelay(uint16_t delay_ms)
{
  uint32_t ticks_per_ms = SystemCoreClock / 1000UL;
  uint32_t wait_ticks = ticks_per_ms * (uint32_t)delay_ms;
  uint32_t start_tick;

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= LED_DWT_CTRL_CYCCNTENA_Msk;

  start_tick = DWT->CYCCNT;
  while ((DWT->CYCCNT - start_tick) < wait_ticks)
  {
  }
}
