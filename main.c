/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv8825.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE  64U
#define UART_RX_QUEUE_DEPTH  4U
#define AUTO_FAST_FREQUENCY_HZ  6400U
#define AUTO_FAST_DURATION_MS   2000U
#define AUTO_SLOW_FREQUENCY_HZ  800U
#define AUTO_SLOW_DURATION_MS   16000U
#define TASK9_TARGET_RPM         60.0f
#define TASK9_HOLD_DURATION_MS   10000U
#define TASK9_UPDATE_INTERVAL_MS 20U
#define TASK10_FULL_TRAVEL_RPM    120.0f
#define TASK10_FULL_TRAVEL_MS     75000U
#define TASK10_FULL_TRAVEL_STEPS  960000UL
#define POSITION_MOVE_RPM          120.0f
#define POSITION_STEP_FREQUENCY_HZ 12800UL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Mechanical speed input in RPM. It can be changed directly from the debugger.
   This project uses a 1.8-degree motor (200 full steps per revolution). */
#define MOTOR_FULL_STEPS_PER_REVOLUTION  200U
volatile float motor_speed_rpm = 30.0f;
static float applied_motor_speed_rpm = 30.0f;
/* Microstep input: valid values are 1, 2, 4, 8, 16 and 32.
   The default value 32 sets MODE0, MODE1 and MODE2 all high. */
volatile uint8_t motor_microstep = 32U;
static uint8_t applied_motor_microstep = 32U;
/* Direction input: 0=FORWARD/up, 1=REVERSE/down. */
volatile uint8_t motor_direction = 1U;
static uint8_t applied_motor_direction = 1U;
/* Run mode, editable in Keil Watch:
   0=disabled, 1=holding, 2=5 Hz diagnostic, 3=100 Hz diagnostic,
   4=slow 800 Hz, 5=normal 3200 Hz, 6=fast 6400 Hz,
   7=custom mechanical speed from motor_speed_rpm,
   8=fast forward, then slow reverse to the start position,
   9=accelerate from 0 to 60 RPM at 10 RPM/s, hold for 10 seconds,
   10=full travel: forward at 120 RPM for 75 seconds (960000 STEP at 32x),
   11=move to a specified 0..100 percent position at 120 RPM. */
/* Start disabled. RUN starts the mode selected by selected_run_mode. */
volatile uint8_t diagnostic_command = 0U;
volatile uint8_t selected_run_mode = 7U;
volatile uint8_t diagnostic_state = 0U;
volatile uint8_t drv8825_fault_active = 0U;
volatile uint32_t drv8825_fault_count = 0U;
static uint8_t applied_diagnostic_command = 0U;
volatile uint32_t uart_command_count = 0U;
volatile uint32_t uart_error_count = 0U;
volatile float motor_current_rpm = 0.0f;
volatile float motor_acceleration_rpm_s = 10.0f;
volatile uint32_t motor_position_steps = 0U;
volatile uint32_t motor_target_position_steps = 0U;
/* 0=idle, 1=mode8 fast FWD, 2=mode8 slow REV,
   3=mode9 accelerating, 4=mode9 holding, 5=mode10 full travel. */
volatile uint8_t auto_cycle_phase = 0U;
static uint32_t auto_cycle_tick = 0U;
static uint32_t auto_update_tick = 0U;
static uint8_t auto_accel_started = 0U;
static uint32_t position_start_steps = 0U;
static uint32_t position_move_steps = 0U;
static uint32_t uart_last_ready_tick = 0U;
static uint8_t uart_rx_byte;
static char uart_rx_buffer[UART_RX_QUEUE_DEPTH][UART_RX_BUFFER_SIZE];
static volatile uint8_t uart_rx_index = 0U;
static volatile uint8_t uart_rx_read = 0U;
static volatile uint8_t uart_rx_write = 0U;
static volatile uint8_t uart_rx_count = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void UART_ProcessCommand(char *command);
static uint8_t UART_ParseUnsigned(const char *text, uint32_t *value);
static uint8_t UART_ParseRpm(const char *text, float *rpm);
static void UART_Send(const char *text);
static void UART_SendStatus(void);
static void UART_SendDebug(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void UART_Send(const char *text)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text,
                         (uint16_t)strlen(text), 100U);
}

static uint8_t UART_ParseUnsigned(const char *text, uint32_t *value)
{
  uint32_t result = 0U;
  uint8_t digits = 0U;

  while (*text == ' ')
  {
    text++;
  }
  while ((*text >= '0') && (*text <= '9'))
  {
    result = (result * 10U) + (uint32_t)(*text - '0');
    digits++;
    text++;
  }
  while (*text == ' ')
  {
    text++;
  }
  if ((digits == 0U) || (*text != '\0'))
  {
    return 0U;
  }
  *value = result;
  return 1U;
}

static uint8_t UART_ParseRpm(const char *text, float *rpm)
{
  uint32_t whole = 0U;
  uint32_t fraction = 0U;
  uint32_t scale = 1U;
  uint32_t rpm_milli;
  uint8_t digits = 0U;
  uint8_t fraction_digits = 0U;

  while (*text == ' ')
  {
    text++;
  }
  while ((*text >= '0') && (*text <= '9'))
  {
    whole = (whole * 10U) + (uint32_t)(*text - '0');
    digits++;
    text++;
  }
  if (*text == '.')
  {
    text++;
    while ((*text >= '0') && (*text <= '9'))
    {
      if (fraction_digits < 3U)
      {
        fraction = (fraction * 10U) + (uint32_t)(*text - '0');
        scale *= 10U;
      }
      fraction_digits++;
      text++;
    }
  }
  while (*text == ' ')
  {
    text++;
  }
  if ((digits == 0U) || (*text != '\0'))
  {
    return 0U;
  }

  rpm_milli = (whole * 1000U) + ((fraction * 1000U) / scale);
  if ((rpm_milli < 10U) || (rpm_milli > 1875000U))
  {
    return 0U;
  }
  *rpm = (float)rpm_milli / 1000.0f;
  return 1U;
}

static void UART_SendStatus(void)
{
  char response[192];
  uint32_t rpm_milli = (uint32_t)(motor_speed_rpm * 1000.0f + 0.5f);
  uint32_t current_rpm_milli =
      (uint32_t)(motor_current_rpm * 1000.0f + 0.5f);
  uint32_t position_centi_percent = (uint32_t)(
      ((uint64_t)motor_position_steps * 10000ULL) /
      (uint64_t)TASK10_FULL_TRAVEL_STEPS);

  (void)sprintf(response,
                "STATUS RPM=%lu.%03lu CURRENT=%lu.%03lu POS=%lu.%02lu%% STEPS=%lu TARGET=%lu MICROSTEP=%u DIR=%u RUNMODE=%u SELECTED=%u STATE=%u PHASE=%u FAULT=%u\r\n",
                rpm_milli / 1000U, rpm_milli % 1000U,
                current_rpm_milli / 1000U, current_rpm_milli % 1000U,
                position_centi_percent / 100U,
                position_centi_percent % 100U,
                motor_position_steps, motor_target_position_steps,
                motor_microstep, motor_direction, diagnostic_command,
                selected_run_mode, diagnostic_state, auto_cycle_phase,
                drv8825_fault_active);
  UART_Send(response);
}

static void UART_SendDebug(void)
{
  char response[160];

  (void)sprintf(response,
                "DEBUG CR1=%04X CCER=%04X BDTR=%04X PSC=%lu ARR=%lu CCR1=%lu GPIOA_ODR=%04X GPIOA_IDR=%04X\r\n",
                (unsigned int)TIM1->CR1, (unsigned int)TIM1->CCER,
                (unsigned int)TIM1->BDTR, (uint32_t)TIM1->PSC,
                (uint32_t)TIM1->ARR, (uint32_t)TIM1->CCR1,
                (unsigned int)GPIOA->ODR, (unsigned int)GPIOA->IDR);
  UART_Send(response);
}

static void UART_ProcessCommand(char *command)
{
  uint32_t value;
  float rpm;

  uart_command_count++;
  if (strncmp(command, "RPM ", 4U) == 0)
  {
    if (UART_ParseRpm(command + 4, &rpm) != 0U)
    {
      motor_speed_rpm = rpm;
      selected_run_mode = 7U;
      if (diagnostic_command >= 2U)
      {
        diagnostic_command = 7U;
      }
      UART_Send("OK, letsgooooo\r\n");
    }
    else
    {
      UART_Send("ERR RPM RANGE 0.01..1875\r\n");
    }
  }
  else if (strncmp(command, "MICROSTEP ", 10U) == 0)
  {
    if ((UART_ParseUnsigned(command + 10, &value) != 0U) &&
        ((value == 1U) || (value == 2U) || (value == 4U) ||
         (value == 8U) || (value == 16U) || (value == 32U)))
    {
      motor_microstep = (uint8_t)value;
      UART_Send("OK, letsgooooo\r\n");
    }
    else
    {
      UART_Send("ERR MICROSTEP 1|2|4|8|16|32\r\n");
    }
  }
  else if (strcmp(command, "DIR FWD") == 0)
  {
    motor_direction = 0U;
    UART_Send("OK, lets stop\r\n");
  }
  else if (strcmp(command, "DIR REV") == 0)
  {
    motor_direction = 1U;
    UART_Send("OK, change directiono\r\n");
  }
  else if (strncmp(command, "RUNMODE ", 8U) == 0)
  {
    if ((UART_ParseUnsigned(command + 8, &value) != 0U) && (value <= 11U))
    {
      selected_run_mode = (uint8_t)value;
      UART_Send("OK, come & go\r\n");
    }
    else
    {
      UART_Send("ERR RUNMODE 0..11\r\n");
    }
  }
  else if (strncmp(command, "GOTO ", 5U) == 0)
  {
    if ((UART_ParseUnsigned(command + 5, &value) != 0U) && (value <= 100U))
    {
      motor_target_position_steps =
          (uint32_t)(((uint64_t)value * TASK10_FULL_TRAVEL_STEPS) / 100ULL);
      selected_run_mode = 11U;
      UART_Send("OK, letsgooooo\r\n");
    }
    else
    {
      UART_Send("ERR GOTO 0..100 PERCENT\r\n");
    }
  }
  else if (strcmp(command, "RUN") == 0)
  {
    diagnostic_command = selected_run_mode;
    UART_Send("OK, letsgooooo\r\n");
  }
  else if (strcmp(command, "STOP") == 0)
  {
    diagnostic_command = 0U;
    UART_Send("OK, lets stop\r\n");
  }
  else if (strcmp(command, "STATUS") == 0)
  {
    UART_SendStatus();
  }
  else if (strcmp(command, "DEBUG") == 0)
  {
    UART_SendDebug();
  }
  else
  {
    UART_Send("ERR UNKNOWN COMMAND\r\n");
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  char uart_command[UART_RX_BUFFER_SIZE];
  uint8_t uart_command_ready;
  uint32_t primask;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
  if (DRV8825_Init(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Continuous forward rotation at the requested speed and subdivision. */
  if (DRV8825_SetMicrostep((DRV8825_Microstep)motor_microstep) != HAL_OK)
  {
    Error_Handler();
  }
  DRV8825_SetDirection((motor_direction == 0U) ?
                       DRV8825_DIRECTION_FORWARD :
                       DRV8825_DIRECTION_REVERSE);
  DRV8825_Disable();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((uart_command_count == 0U) &&
        ((HAL_GetTick() - uart_last_ready_tick) >= 1000U))
    {
      uart_last_ready_tick = HAL_GetTick();
      UART_Send("READY DRV8825 RPM CONTROL\r\n");
    }

    uart_command_ready = 0U;
    primask = __get_PRIMASK();
    __disable_irq();
    if (uart_rx_count != 0U)
    {
      (void)strcpy(uart_command, uart_rx_buffer[uart_rx_read]);
      uart_rx_read = (uint8_t)((uart_rx_read + 1U) % UART_RX_QUEUE_DEPTH);
      uart_rx_count--;
      uart_command_ready = 1U;
    }
    if (primask == 0U)
    {
      __enable_irq();
    }
    if (uart_command_ready != 0U)
    {
      UART_ProcessCommand(uart_command);
    }

    drv8825_fault_active =
        (HAL_GPIO_ReadPin(MOTOR_FAULT_GPIO_Port, MOTOR_FAULT_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if (drv8825_fault_active != 0U)
    {
      (void)DRV8825_Stop();
      DRV8825_Disable();
      diagnostic_state = 9U; /* FAULT */
      diagnostic_command = 0U;
      applied_diagnostic_command = 0U;
      drv8825_fault_count++;
      HAL_Delay(10U);
      continue;
    }

    if (diagnostic_command != applied_diagnostic_command)
    {
      (void)DRV8825_Stop();
      DRV8825_Disable();
      if ((diagnostic_command != 8U) && (diagnostic_command != 9U) &&
          (diagnostic_command != 10U) && (diagnostic_command != 11U))
      {
        auto_cycle_phase = 0U;
        auto_accel_started = 0U;
        motor_current_rpm = 0.0f;
      }

      switch (diagnostic_command)
      {
        case 0U:
          diagnostic_state = 0U; /* Disabled */
          break;
        case 1U:
          DRV8825_Enable();
          diagnostic_state = 1U; /* Holding, no STEP */
          break;
        case 2U:
          if (DRV8825_Start(5U) != HAL_OK)
          {
            Error_Handler();
          }
          diagnostic_state = 2U; /* Slow run */
          break;
        case 3U:
          if (DRV8825_Start(100U) != HAL_OK)
          {
            Error_Handler();
          }
          diagnostic_state = 3U; /* 100 Hz diagnostic */
          break;
        case 4U:
          if (DRV8825_Start(800U) != HAL_OK)
          {
            Error_Handler();
          }
          diagnostic_state = 4U; /* Slow */
          break;
        case 5U:
          if (DRV8825_Start(3200U) != HAL_OK)
          {
            Error_Handler();
          }
          diagnostic_state = 5U; /* Normal */
          break;
        case 6U:
          if (DRV8825_Start(6400U) != HAL_OK)
          {
            Error_Handler();
          }
          diagnostic_state = 6U; /* Fast */
          break;
        case 7U:
          if (DRV8825_StartRPM(motor_speed_rpm,
                               MOTOR_FULL_STEPS_PER_REVOLUTION,
                               motor_microstep) != HAL_OK)
          {
            Error_Handler();
          }
          diagnostic_state = 7U; /* Custom */
          break;
        case 8U:
          DRV8825_SetDirection(DRV8825_DIRECTION_FORWARD);
          motor_direction = 0U;
          applied_motor_direction = 0U;
          if (DRV8825_Start(AUTO_FAST_FREQUENCY_HZ) != HAL_OK)
          {
            Error_Handler();
          }
          auto_cycle_phase = 1U;
          auto_cycle_tick = HAL_GetTick();
          diagnostic_state = 8U; /* Automatic fast-forward phase */
          break;
        case 9U:
          DRV8825_SetDirection(DRV8825_DIRECTION_FORWARD);
          motor_direction = 0U;
          applied_motor_direction = 0U;
          DRV8825_Enable();
          motor_current_rpm = 0.0f;
          auto_accel_started = 0U;
          auto_cycle_phase = 3U;
          auto_cycle_tick = HAL_GetTick();
          auto_update_tick = auto_cycle_tick;
          diagnostic_state = 11U; /* Task 9 accelerating */
          break;
        case 10U:
          DRV8825_SetDirection(DRV8825_DIRECTION_FORWARD);
          motor_direction = 0U;
          applied_motor_direction = 0U;
          if (DRV8825_StartRPM(TASK10_FULL_TRAVEL_RPM,
                               MOTOR_FULL_STEPS_PER_REVOLUTION,
                               motor_microstep) != HAL_OK)
          {
            Error_Handler();
          }
          motor_current_rpm = TASK10_FULL_TRAVEL_RPM;
          auto_cycle_phase = 5U;
          auto_cycle_tick = HAL_GetTick();
          diagnostic_state = 13U; /* Task 10 full-travel run */
          break;
        case 11U:
          if (motor_target_position_steps == motor_position_steps)
          {
            diagnostic_command = 0U;
            diagnostic_state = 0U;
            auto_cycle_phase = 0U;
            break;
          }
          if (DRV8825_SetMicrostep(DRV8825_MICROSTEP_32) != HAL_OK)
          {
            Error_Handler();
          }
          motor_microstep = 32U;
          applied_motor_microstep = 32U;
          position_start_steps = motor_position_steps;
          if (motor_target_position_steps > motor_position_steps)
          {
            position_move_steps =
                motor_target_position_steps - motor_position_steps;
            DRV8825_SetDirection(DRV8825_DIRECTION_FORWARD);
            motor_direction = 0U;
            applied_motor_direction = 0U;
          }
          else
          {
            position_move_steps =
                motor_position_steps - motor_target_position_steps;
            DRV8825_SetDirection(DRV8825_DIRECTION_REVERSE);
            motor_direction = 1U;
            applied_motor_direction = 1U;
          }
          if (DRV8825_StartRPM(POSITION_MOVE_RPM,
                               MOTOR_FULL_STEPS_PER_REVOLUTION,
                               motor_microstep) != HAL_OK)
          {
            Error_Handler();
          }
          motor_current_rpm = POSITION_MOVE_RPM;
          auto_cycle_phase = 6U;
          auto_cycle_tick = HAL_GetTick();
          diagnostic_state = 14U; /* Moving to specified position */
          break;
        default:
          diagnostic_command = 0U;
          diagnostic_state = 8U; /* Invalid command */
          break;
      }
      applied_diagnostic_command = diagnostic_command;
    }

    /* Mode 8 uses equal STEP counts in both directions: 6400 x 2 seconds
       forward, then 800 x 16 seconds reverse. It remains UART-responsive. */
    if ((diagnostic_command == 8U) && (auto_cycle_phase == 1U) &&
        ((HAL_GetTick() - auto_cycle_tick) >= AUTO_FAST_DURATION_MS))
    {
      (void)DRV8825_Stop();
      DRV8825_SetDirection(DRV8825_DIRECTION_REVERSE);
      motor_direction = 1U;
      applied_motor_direction = 1U;
      if (DRV8825_Start(AUTO_SLOW_FREQUENCY_HZ) != HAL_OK)
      {
        Error_Handler();
      }
      auto_cycle_phase = 2U;
      auto_cycle_tick = HAL_GetTick();
      diagnostic_state = 10U; /* Automatic slow-reverse phase */
    }
    else if ((diagnostic_command == 8U) && (auto_cycle_phase == 2U) &&
             ((HAL_GetTick() - auto_cycle_tick) >= AUTO_SLOW_DURATION_MS))
    {
      (void)DRV8825_Stop();
      DRV8825_Disable();
      auto_cycle_phase = 0U;
      diagnostic_command = 0U;
      applied_diagnostic_command = 0U;
      diagnostic_state = 0U;
    }

    /* Task 9: ramp from 0 to 60 RPM at the Watch-adjustable acceleration
       (default 10 RPM/s), hold 60 RPM for 10 seconds, then stop. */
    if ((diagnostic_command == 9U) && (auto_cycle_phase == 3U) &&
        ((HAL_GetTick() - auto_update_tick) >= TASK9_UPDATE_INTERVAL_MS))
    {
      motor_current_rpm = motor_acceleration_rpm_s *
                          (float)(HAL_GetTick() - auto_cycle_tick) / 1000.0f;
      if (motor_current_rpm >= TASK9_TARGET_RPM)
      {
        motor_current_rpm = TASK9_TARGET_RPM;
      }

      if (auto_accel_started == 0U)
      {
        if (DRV8825_StartRPM(motor_current_rpm,
                             MOTOR_FULL_STEPS_PER_REVOLUTION,
                             motor_microstep) != HAL_OK)
        {
          Error_Handler();
        }
        auto_accel_started = 1U;
      }
      else if (DRV8825_SetSpeedRPM(motor_current_rpm,
                                   MOTOR_FULL_STEPS_PER_REVOLUTION,
                                   motor_microstep) != HAL_OK)
      {
        Error_Handler();
      }
      auto_update_tick = HAL_GetTick();

      if (motor_current_rpm >= TASK9_TARGET_RPM)
      {
        auto_cycle_phase = 4U;
        auto_cycle_tick = HAL_GetTick();
        diagnostic_state = 12U; /* Task 9 holding 60 RPM */
      }
    }
    else if ((diagnostic_command == 9U) && (auto_cycle_phase == 4U) &&
             ((HAL_GetTick() - auto_cycle_tick) >= TASK9_HOLD_DURATION_MS))
    {
      (void)DRV8825_Stop();
      DRV8825_Disable();
      motor_current_rpm = 0.0f;
      auto_accel_started = 0U;
      auto_cycle_phase = 0U;
      diagnostic_command = 0U;
      applied_diagnostic_command = 0U;
      diagnostic_state = 0U;
    }

    /* Task 10 full travel calibration: at 120 RPM and 32 microsteps,
       75 seconds produces 960000 STEP pulses (150 motor revolutions). */
    if ((diagnostic_command == 10U) && (auto_cycle_phase == 5U) &&
        ((HAL_GetTick() - auto_cycle_tick) >= TASK10_FULL_TRAVEL_MS))
    {
      (void)DRV8825_Stop();
      DRV8825_Disable();
      motor_current_rpm = 0.0f;
      auto_cycle_phase = 0U;
      diagnostic_command = 0U;
      applied_diagnostic_command = 0U;
      diagnostic_state = 0U;
    }

    /* Mode 11 estimates position from the exact commanded STEP rate. */
    if ((diagnostic_command == 11U) && (auto_cycle_phase == 6U))
    {
      uint32_t travelled_steps = (uint32_t)(
          ((uint64_t)(HAL_GetTick() - auto_cycle_tick) *
           POSITION_STEP_FREQUENCY_HZ) / 1000ULL);
      if (travelled_steps >= position_move_steps)
      {
        (void)DRV8825_Stop();
        DRV8825_Disable();
        motor_position_steps = motor_target_position_steps;
        motor_current_rpm = 0.0f;
        auto_cycle_phase = 0U;
        diagnostic_command = 0U;
        applied_diagnostic_command = 0U;
        diagnostic_state = 0U;
      }
      else if (motor_target_position_steps > position_start_steps)
      {
        motor_position_steps = position_start_steps + travelled_steps;
      }
      else
      {
        motor_position_steps = position_start_steps - travelled_steps;
      }
    }

    /* In custom mode, apply a new mechanical speed whenever motor_speed_rpm
       is changed. The driver converts RPM to STEP/s using the subdivision. */
    if ((diagnostic_command == 7U) &&
        (motor_speed_rpm != applied_motor_speed_rpm))
    {
      if (DRV8825_SetSpeedRPM(motor_speed_rpm,
                              MOTOR_FULL_STEPS_PER_REVOLUTION,
                              motor_microstep) == HAL_OK)
      {
        applied_motor_speed_rpm = motor_speed_rpm;
      }
    }

    /* Apply a new subdivision value. Invalid values are ignored. */
    if (motor_microstep != applied_motor_microstep)
    {
      if (DRV8825_SetMicrostep((DRV8825_Microstep)motor_microstep) == HAL_OK)
      {
        applied_motor_microstep = motor_microstep;
        /* Re-enter the selected mode. Custom mode recalculates STEP/s from
           RPM so changing subdivision does not change mechanical speed. */
        applied_diagnostic_command = 255U;
      }
    }

    /* Stop before changing DIR, then restart the selected run mode on the
       next loop iteration. Values other than 0 or 1 are ignored. */
    if ((motor_direction != applied_motor_direction) &&
        (motor_direction <= 1U))
    {
      (void)DRV8825_Stop();
      DRV8825_SetDirection((motor_direction == 0U) ?
                           DRV8825_DIRECTION_FORWARD :
                           DRV8825_DIRECTION_REVERSE);
      applied_motor_direction = motor_direction;
      applied_diagnostic_command = 255U;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, MOTOR_EN_Pin|MOTOR_MODE0_Pin|MOTOR_MODE1_Pin
                          |MOTOR_MODE2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : MOTOR_FAULT_Pin */
  GPIO_InitStruct.Pin = MOTOR_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MOTOR_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_DIR_Pin MOTOR_EN_Pin MOTOR_MODE0_Pin MOTOR_MODE1_Pin
                           MOTOR_MODE2_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_Pin|MOTOR_EN_Pin|MOTOR_MODE0_Pin|MOTOR_MODE1_Pin
                          |MOTOR_MODE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (uart_rx_count < UART_RX_QUEUE_DEPTH)
    {
      if ((uart_rx_byte == '\r') || (uart_rx_byte == '\n'))
      {
        if (uart_rx_index != 0U)
        {
          uart_rx_buffer[uart_rx_write][uart_rx_index] = '\0';
          uart_rx_index = 0U;
          uart_rx_write = (uint8_t)((uart_rx_write + 1U) % UART_RX_QUEUE_DEPTH);
          uart_rx_count++;
        }
      }
      else if (uart_rx_index < (UART_RX_BUFFER_SIZE - 1U))
      {
        uart_rx_buffer[uart_rx_write][uart_rx_index] = (char)uart_rx_byte;
        uart_rx_index++;
      }
      else
      {
        uart_rx_index = 0U;
        uart_error_count++;
      }
    }
    else if ((uart_rx_byte == '\r') || (uart_rx_byte == '\n'))
    {
      uart_rx_index = 0U;
      uart_error_count++;
    }
    if (HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U) != HAL_OK)
    {
      uart_error_count++;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uart_error_count++;
    (void)HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
