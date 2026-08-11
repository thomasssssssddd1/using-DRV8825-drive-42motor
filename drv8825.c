#include "drv8825.h"

static TIM_HandleTypeDef *drv_timer;
static uint32_t drv_channel;
static uint8_t drv_running;

static uint32_t DRV8825_GetTimerClock(void)
{
  uint32_t timer_clock = HAL_RCC_GetPCLK2Freq();

  /* STM32F1 timers run at 2 x PCLK when the APB prescaler is not 1. */
  if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_HCLK_DIV1)
  {
    timer_clock *= 2U;
  }
  return timer_clock;
}

HAL_StatusTypeDef DRV8825_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
  if ((htim == NULL) || (htim->Instance != TIM1) ||
      (channel != TIM_CHANNEL_1))
  {
    return HAL_ERROR;
  }

  drv_timer = htim;
  drv_channel = channel;
  drv_running = 0U;
  DRV8825_Disable();
  DRV8825_SetDirection(DRV8825_DIRECTION_FORWARD);
  (void)DRV8825_SetMicrostep(DRV8825_MICROSTEP_32);
  return HAL_TIM_PWM_Stop(drv_timer, drv_channel);
}

HAL_StatusTypeDef DRV8825_SetStepFrequency(uint32_t step_frequency_hz)
{
  uint32_t timer_clock;
  uint32_t divider;
  uint32_t period_counts;
  uint8_t restart;

  if ((drv_timer == NULL) || (step_frequency_hz == 0U) ||
      (step_frequency_hz > DRV8825_MAX_STEP_FREQUENCY_HZ))
  {
    return HAL_ERROR;
  }

  timer_clock = DRV8825_GetTimerClock();
  divider = (uint32_t)(((uint64_t)timer_clock +
            ((uint64_t)step_frequency_hz * 65536ULL) - 1ULL) /
            ((uint64_t)step_frequency_hz * 65536ULL));
  if (divider == 0U)
  {
    divider = 1U;
  }
  if (divider > 65536U)
  {
    return HAL_ERROR;
  }

  period_counts = timer_clock / divider / step_frequency_hz;
  if ((period_counts < 4U) || (period_counts > 65536U))
  {
    return HAL_ERROR;
  }

  restart = drv_running;
  if (restart != 0U)
  {
    (void)HAL_TIM_PWM_Stop(drv_timer, drv_channel);
  }

  drv_timer->Instance->PSC = divider - 1U;
  drv_timer->Instance->ARR = period_counts - 1U;
  __HAL_TIM_SET_COMPARE(drv_timer, drv_channel, period_counts / 2U);
  drv_timer->Instance->EGR = TIM_EGR_UG;

  if (restart != 0U)
  {
    return HAL_TIM_PWM_Start(drv_timer, drv_channel);
  }
  return HAL_OK;
}

HAL_StatusTypeDef DRV8825_Start(uint32_t step_frequency_hz)
{
  HAL_StatusTypeDef status;

  status = DRV8825_SetStepFrequency(step_frequency_hz);
  if (status != HAL_OK)
  {
    return status;
  }

  DRV8825_Enable();
  /* Far exceeds the DRV8825 650 ns nENBL-to-STEP setup requirement. */
  HAL_Delay(1U);
  status = HAL_TIM_PWM_Start(drv_timer, drv_channel);
  if (status == HAL_OK)
  {
    drv_running = 1U;
  }
  return status;
}

HAL_StatusTypeDef DRV8825_StartRPM(float rpm,
                                  uint16_t full_steps_per_revolution,
                                  uint8_t microstep_divisor)
{
  float step_frequency;

  if ((rpm <= 0.0f) || (full_steps_per_revolution == 0U) ||
      (microstep_divisor == 0U))
  {
    return HAL_ERROR;
  }

  step_frequency = rpm * (float)full_steps_per_revolution *
                   (float)microstep_divisor / 60.0f;
  if ((step_frequency < 1.0f) ||
      (step_frequency > (float)DRV8825_MAX_STEP_FREQUENCY_HZ))
  {
    return HAL_ERROR;
  }
  return DRV8825_Start((uint32_t)(step_frequency + 0.5f));
}

HAL_StatusTypeDef DRV8825_Stop(void)
{
  HAL_StatusTypeDef status;

  if (drv_timer == NULL)
  {
    return HAL_ERROR;
  }
  status = HAL_TIM_PWM_Stop(drv_timer, drv_channel);
  drv_running = 0U;
  return status;
}

HAL_StatusTypeDef DRV8825_MoveSteps(uint32_t steps,
                                   uint32_t step_frequency_hz)
{
  uint32_t completed_steps;

  if ((steps == 0U) || (drv_timer == NULL))
  {
    return HAL_ERROR;
  }
  if (DRV8825_SetStepFrequency(step_frequency_hz) != HAL_OK)
  {
    return HAL_ERROR;
  }

  DRV8825_Enable();
  HAL_Delay(1U);
  __HAL_TIM_CLEAR_FLAG(drv_timer, TIM_FLAG_CC1);
  if (HAL_TIM_PWM_Start(drv_timer, drv_channel) != HAL_OK)
  {
    return HAL_ERROR;
  }
  drv_running = 1U;

  /* Each CC1 event is the falling edge of one complete STEP pulse. Waiting
     for exactly N events and then stopping produces exactly N rising edges. */
  for (completed_steps = 0U; completed_steps < steps; completed_steps++)
  {
    while (__HAL_TIM_GET_FLAG(drv_timer, TIM_FLAG_CC1) == RESET)
    {
    }
    __HAL_TIM_CLEAR_FLAG(drv_timer, TIM_FLAG_CC1);
  }

  return DRV8825_Stop();
}

HAL_StatusTypeDef DRV8825_MoveDegrees(float degrees,
                                     uint32_t step_frequency_hz,
                                     uint16_t full_steps_per_revolution,
                                     DRV8825_Microstep microstep)
{
  float step_count;

  if ((degrees <= 0.0f) || (full_steps_per_revolution == 0U))
  {
    return HAL_ERROR;
  }
  if (DRV8825_SetMicrostep(microstep) != HAL_OK)
  {
    return HAL_ERROR;
  }

  step_count = degrees * (float)full_steps_per_revolution *
               (float)microstep / 360.0f;
  if ((step_count < 1.0f) || (step_count > 4294967040.0f))
  {
    return HAL_ERROR;
  }
  return DRV8825_MoveSteps((uint32_t)(step_count + 0.5f),
                           step_frequency_hz);
}

HAL_StatusTypeDef DRV8825_SetSpeedRPM(float rpm,
                                     uint16_t full_steps_per_revolution,
                                     uint8_t microstep_divisor)
{
  float step_frequency;

  if ((rpm <= 0.0f) || (full_steps_per_revolution == 0U) ||
      (microstep_divisor == 0U))
  {
    return HAL_ERROR;
  }

  step_frequency = rpm * (float)full_steps_per_revolution *
                   (float)microstep_divisor / 60.0f;
  if ((step_frequency < 1.0f) ||
      (step_frequency > (float)DRV8825_MAX_STEP_FREQUENCY_HZ))
  {
    return HAL_ERROR;
  }
  return DRV8825_SetStepFrequency((uint32_t)(step_frequency + 0.5f));
}

void DRV8825_SetDirection(DRV8825_Direction direction)
{
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin,
                    (direction == DRV8825_DIRECTION_REVERSE) ?
                    GPIO_PIN_SET : GPIO_PIN_RESET);
}

HAL_StatusTypeDef DRV8825_SetMicrostep(DRV8825_Microstep microstep)
{
  GPIO_PinState mode0 = GPIO_PIN_RESET;
  GPIO_PinState mode1 = GPIO_PIN_RESET;
  GPIO_PinState mode2 = GPIO_PIN_RESET;

  switch (microstep)
  {
    case DRV8825_MICROSTEP_FULL:
      break;
    case DRV8825_MICROSTEP_2:
      mode0 = GPIO_PIN_SET;
      break;
    case DRV8825_MICROSTEP_4:
      mode1 = GPIO_PIN_SET;
      break;
    case DRV8825_MICROSTEP_8:
      mode0 = GPIO_PIN_SET;
      mode1 = GPIO_PIN_SET;
      break;
    case DRV8825_MICROSTEP_16:
      mode2 = GPIO_PIN_SET;
      break;
    case DRV8825_MICROSTEP_32:
      mode0 = GPIO_PIN_SET;
      mode1 = GPIO_PIN_SET;
      mode2 = GPIO_PIN_SET;
      break;
    default:
      return HAL_ERROR;
  }

  HAL_GPIO_WritePin(MOTOR_MODE0_GPIO_Port, MOTOR_MODE0_Pin, mode0);
  HAL_GPIO_WritePin(MOTOR_MODE1_GPIO_Port, MOTOR_MODE1_Pin, mode1);
  HAL_GPIO_WritePin(MOTOR_MODE2_GPIO_Port, MOTOR_MODE2_Pin, mode2);
  return HAL_OK;
}

void DRV8825_Enable(void)
{
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
}

void DRV8825_Disable(void)
{
  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
}
