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
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include "stm32f4xx_hal_adc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "arm_kinematics.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  TIM_HandleTypeDef *htim;
  uint32_t tim_channel;
  GPIO_TypeDef *dir_port;
  uint16_t dir_pin;
  GPIO_TypeDef *en_port;
  uint16_t en_pin;
  uint8_t enabled;
  uint8_t direction;
  uint32_t rate_hz;
  uint8_t running;
  uint32_t move_target_steps;
  uint32_t move_done_steps;
  uint8_t move_active;
  uint32_t move_rate_hz;
  uint32_t current_rate_hz;
  uint32_t ramp_last_tick;
  volatile int32_t position_steps;
  uint8_t move_ramp;
} stepper_motor_t;

typedef enum
{
  ARM_MODE_IDLE = 0,
  ARM_MODE_JOYSTICK,
  ARM_MODE_GOTO,
  ARM_MODE_JMOVE,
  ARM_MODE_SETTLE
} arm_mode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PKT_SYNC_0                0xA5U
#define PKT_SYNC_1                0x5AU
#define PKT_TYPE_TEXT             3U
#define PKT_TYPE_BIN_CMD          16U
#define PKT_TYPE_ACK              0xFEU
#define PKT_TYPE_NACK             0xFFU
#define RX_BUFFER_MAX             256U
#define STATUS_TEXT_PERIOD_MS     1000U
#define TX_TEXT_BUFFER_SIZE       128U
#define STEPPER_TIMER_CLK_HZ      1000000U
#define STEPPER_DEFAULT_RATE_HZ   200U
#define STEPPER_MOVE_RATE_HZ      200U
#define STEPPER_MAX_RATE_HZ       5000U
#define STEPPER_COUNT             4U
#define STEPPER_STEPS_PER_REV     200U
#define STEPPER_MAX_MOVE_DEG      8640U
#define STEPPER_START_RATE_HZ     120U
#define STEPPER_RAMP_STEP_HZ      20U
#define STEPPER_RAMP_INTERVAL_MS  20U
#define STEPPER_DECEL_STEPS       100U
#define JOYSTICK_AXIS_COUNT       4U
#define JOYSTICK_DEADZONE         250U
#define JOYSTICK_CALIBRATION_SAMPLES 16U

/* Joystick axis order: 0 = left X, 1 = left Y, 2 = right X, 3 = right Y */
#define JOY_LEFT_X                0U
#define JOY_LEFT_Y                1U
#define JOY_RIGHT_X               2U
#define JOY_RIGHT_Y               3U

/* Gearing (full step: MS1/MS2/MS3 tied to GND). Change ARM_MICROSTEP if the driver mode changes. */
#define ARM_MICROSTEP             1.0f
#define ARM_STATUS_PERIOD_MS      500U

/* Arm motion engine: every tick each joint is given a short burst of whole steps. */
#define ARM_TICK_MS               20U
#define ARM_MAX_JOINT_RATE_HZ     500.0f
#define ARM_MAX_BURST_STEPS       12U
#define ARM_MAX_LAG_STEPS         4.0f
#define ARM_STEP_THRESHOLD        0.6f
#define ARM_SPEED_DEFAULT_MM_S    20.0f
#define ARM_SPEED_MAX_MM_S        60.0f
#define ARM_TIP_ACCEL_MM_S2       80.0f
#define ARM_JMOVE_MAX_RATE_HZ     250.0f
#define ARM_JMOVE_MIN_RATE_HZ     30.0f
#define ARM_JMOVE_ACCEL_HZ_S      300.0f
#define ARM_POINTER_TOLERANCE_DEG 1.0f
#define ARM_READY_X_MM            0.0f
#define ARM_READY_Y_MM            200.0f
#define ARM_READY_Z_MM            200.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t uart_tx_ready = 1;
volatile uint8_t rx_packet_ready = 0;
volatile uint8_t rx_byte = 0;

uint32_t next_status_text_tick = 0;
ADC_HandleTypeDef hadc1;
uint16_t joystick_filtered_values[JOYSTICK_AXIS_COUNT] = {2048U, 2048U, 2048U, 2048U};
uint16_t joystick_raw_values[JOYSTICK_AXIS_COUNT] = {2048U, 2048U, 2048U, 2048U};
uint16_t joystick_center_values[JOYSTICK_AXIS_COUNT] = {2048U, 2048U, 2048U, 2048U};
/* ADC1 channels: PC1 = IN11, PC2 = IN12, PC3 = IN13, PC4 = IN14 */
static const uint32_t joystick_channels[JOYSTICK_AXIS_COUNT] = {ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14};
/* Flip an entry to -1 if a stick moves the tip the wrong way (also settable with JOYSIGN=a,b,c,d). */
static float joystick_sign[JOYSTICK_AXIS_COUNT] = {1.0f, 1.0f, 1.0f, 1.0f};

/* Joint order: base, shoulder, elbow, wrist (motors 1..4) */
static const float joint_gear_ratio[STEPPER_COUNT] = {17.0f, 25.0f, 25.0f, 25.0f};
/* +1 if DIR = 1 increases the joint angle, -1 if it decreases it. Flip per joint after checking with JMOVE. */
static const float joint_dir_sign[STEPPER_COUNT] = {1.0f, 1.0f, -1.0f, 1.0f};

static arm_mode_t arm_mode = ARM_MODE_IDLE;
static uint8_t arm_homed = 0U;
static uint8_t arm_ready_pending = 0U;
static uint32_t next_arm_tick = 0U;
static uint32_t next_arm_message_tick = 0U;
static float arm_speed_mm_s = ARM_SPEED_DEFAULT_MM_S;
static arm_point_t tip_target = {0.0f, 0.0f, 0.0f};
static float tip_velocity[3] = {0.0f, 0.0f, 0.0f};
static float base_target_deg = 0.0f;
static float joint_target_raw[STEPPER_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static float ik_offset_raw[STEPPER_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f};
static arm_point_t goto_goal = {0.0f, 0.0f, 0.0f};
static float jmove_start_raw[STEPPER_COUNT];
static float jmove_goal_raw[STEPPER_COUNT];
static float jmove_distance = 0.0f;
static float jmove_progress = 0.0f;
static float jmove_speed = 0.0f;
static float goto_speed = 0.0f;
static char arm_message[TX_TEXT_BUFFER_SIZE];
static uint8_t arm_message_pending = 0U;
static uint8_t pose_status_turn = 0U;

uint8_t tx_text_packet[2U + 1U + 2U + TX_TEXT_BUFFER_SIZE];
uint8_t tx_ack_packet[5U];
uint8_t tx_nack_packet[5U];

uint8_t rx_buffer[RX_BUFFER_MAX];
volatile uint16_t rx_index = 0;
volatile uint16_t rx_frame_len = 0;

stepper_motor_t motors[STEPPER_COUNT] = {
  {&htim2, TIM_CHANNEL_1, GPIOA, GPIO_PIN_13, GPIOA, GPIO_PIN_14, 0U, 0U, STEPPER_DEFAULT_RATE_HZ, 0U, 0U, 0U, 0U, STEPPER_DEFAULT_RATE_HZ, 0U, 0U, 0, 1U},
  {&htim5, TIM_CHANNEL_2, GPIOA, GPIO_PIN_4,  GPIOA, GPIO_PIN_0,  0U, 0U, STEPPER_DEFAULT_RATE_HZ, 0U, 0U, 0U, 0U, STEPPER_DEFAULT_RATE_HZ, 0U, 0U, 0, 1U},
  {&htim10, TIM_CHANNEL_1, GPIOC, GPIO_PIN_9,  GPIOA, GPIO_PIN_6,  0U, 0U, STEPPER_DEFAULT_RATE_HZ, 0U, 0U, 0U, 0U, STEPPER_DEFAULT_RATE_HZ, 0U, 0U, 0, 1U},
  {&htim4, TIM_CHANNEL_1, GPIOB, GPIO_PIN_3,  GPIOA, GPIO_PIN_7,  0U, 0U, STEPPER_DEFAULT_RATE_HZ, 0U, 0U, 0U, 0U, STEPPER_DEFAULT_RATE_HZ, 0U, 0U, 0, 1U}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void stepper_set_step_rate_hz(uint8_t motor_id, uint32_t rate_hz);
static stepper_motor_t *get_motor_by_id(uint8_t motor_id);
static void stepper_set_direction(uint8_t motor_id, uint8_t direction);
static void joystick_init(void);
static uint16_t joystick_read(uint32_t channel);
static uint16_t joystick_calibrate_center(uint32_t channel);
static void send_joystick_status(void);
static void arm_halt(void);
static void arm_tick(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void start_uart_rx_it(void)
{
  if (!rx_packet_ready)
  {
    (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1U);
  }
}

static void joystick_init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1U;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
}

static uint16_t joystick_read(uint32_t channel)
{
  ADC_ChannelConfTypeDef channel_config = {0};

  channel_config.Channel = channel;
  channel_config.Rank = 1U;
  channel_config.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &channel_config) != HAL_OK ||
      HAL_ADC_Start(&hadc1) != HAL_OK ||
      HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return 2048U;
  }

  uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return value;
}

static uint16_t joystick_calibrate_center(uint32_t channel)
{
  uint32_t total = 0U;
  uint8_t sample;

  (void)joystick_read(channel);
  for (sample = 0U; sample < JOYSTICK_CALIBRATION_SAMPLES; sample++)
  {
    total += joystick_read(channel);
  }

  return (uint16_t)(total / JOYSTICK_CALIBRATION_SAMPLES);
}

static uint16_t build_text_packet(uint8_t *dst, const char *text)
{
  uint16_t idx = 0;
  uint16_t text_len = (uint16_t)strlen(text);

  if (text_len > TX_TEXT_BUFFER_SIZE)
  {
    text_len = TX_TEXT_BUFFER_SIZE;
  }

  dst[idx++] = PKT_SYNC_0;
  dst[idx++] = PKT_SYNC_1;
  dst[idx++] = PKT_TYPE_TEXT;
  dst[idx++] = (uint8_t)(text_len & 0xFFU);
  dst[idx++] = (uint8_t)((text_len >> 8) & 0xFFU);

  memcpy(&dst[idx], text, text_len);
  idx = (uint16_t)(idx + text_len);

  return idx;
}

static void send_ack(uint8_t ack_type)
{
  uint8_t *packet = tx_nack_packet;

  if (ack_type == PKT_TYPE_ACK)
  {
    packet = tx_ack_packet;
  }

  packet[0] = PKT_SYNC_0;
  packet[1] = PKT_SYNC_1;
  packet[2] = ack_type;
  packet[3] = 0U;
  packet[4] = 0U;

  if (HAL_UART_Transmit_IT(&huart2, packet, 5U) == HAL_OK)
  {
    uart_tx_ready = 0;
  }
}

static stepper_motor_t *get_motor_by_id(uint8_t motor_id)
{
  if ((motor_id == 0U) || (motor_id > STEPPER_COUNT))
  {
    return NULL;
  }

  return &motors[motor_id - 1U];
}

static void stepper_start_pwm(stepper_motor_t *motor)
{
  if (motor->running == 0U)
  {
    if (HAL_TIM_PWM_Start_IT(motor->htim, motor->tim_channel) == HAL_OK)
    {
      motor->running = 1U;
    }
    else
    {
      Error_Handler();
    }
  }
}

static void stepper_stop_pwm(stepper_motor_t *motor)
{
  if (motor->running != 0U)
  {
    (void)HAL_TIM_PWM_Stop_IT(motor->htim, motor->tim_channel);
    motor->running = 0U;
  }
}

static uint8_t stepper_move_degrees(uint8_t motor_id, uint32_t degrees)
{
  uint8_t idx;
  uint32_t steps;

  if (degrees > STEPPER_MAX_MOVE_DEG)
  {
    return 0U;
  }

  steps = ((degrees * STEPPER_STEPS_PER_REV) + 180U) / 360U;
  if ((degrees > 0U) && (steps == 0U))
  {
    steps = 1U;
  }

  if (motor_id == 0U)
  {
    for (idx = 0U; idx < STEPPER_COUNT; idx++)
    {
      motors[idx].move_target_steps = steps;
      motors[idx].move_done_steps = 0U;
      motors[idx].move_ramp = 1U;
      motors[idx].move_active = (steps > 0U) ? 1U : 0U;
      if (motors[idx].move_rate_hz == 0U)
      {
        motors[idx].move_rate_hz = STEPPER_MOVE_RATE_HZ;
      }
      motors[idx].current_rate_hz = (motors[idx].move_rate_hz < STEPPER_START_RATE_HZ) ? motors[idx].move_rate_hz : STEPPER_START_RATE_HZ;
      motors[idx].ramp_last_tick = HAL_GetTick();
      stepper_set_step_rate_hz((uint8_t)(idx + 1U), (steps > 0U) ? motors[idx].current_rate_hz : 0U);
    }
    return 1U;
  }

  stepper_motor_t *motor = get_motor_by_id(motor_id);
  if (motor == NULL)
  {
    return 0U;
  }

  motor->move_target_steps = steps;
  motor->move_done_steps = 0U;
  motor->move_ramp = 1U;
  motor->move_active = (steps > 0U) ? 1U : 0U;
  if (motor->move_rate_hz == 0U)
  {
    motor->move_rate_hz = STEPPER_MOVE_RATE_HZ;
  }
  motor->current_rate_hz = (motor->move_rate_hz < STEPPER_START_RATE_HZ) ? motor->move_rate_hz : STEPPER_START_RATE_HZ;
  motor->ramp_last_tick = HAL_GetTick();
  stepper_set_step_rate_hz(motor_id, (steps > 0U) ? motor->current_rate_hz : 0U);

  return 1U;
}

static void stepper_apply_outputs(stepper_motor_t *motor)
{
  HAL_GPIO_WritePin(motor->en_port, motor->en_pin,
                    (motor->enabled != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin,
                    (motor->direction != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if ((motor->enabled == 0U) || (motor->rate_hz == 0U))
  {
    stepper_stop_pwm(motor);
  }
  else
  {
    stepper_start_pwm(motor);
  }
}

static void stepper_set_enable(uint8_t motor_id, uint8_t enable)
{
  uint8_t idx;

  if (motor_id == 0U)
  {
    for (idx = 0U; idx < STEPPER_COUNT; idx++)
    {
      motors[idx].enabled = (enable != 0U) ? 1U : 0U;
      if (motors[idx].enabled != 0U)
      {
        motors[idx].rate_hz = 0U;
        motors[idx].current_rate_hz = 0U;
        motors[idx].move_active = 0U;
      }
      else
      {
        motors[idx].move_active = 0U;
      }
      stepper_apply_outputs(&motors[idx]);
    }
    return;
  }

  stepper_motor_t *motor = get_motor_by_id(motor_id);
  if (motor != NULL)
  {
    motor->enabled = (enable != 0U) ? 1U : 0U;
    if (motor->enabled != 0U)
    {
      motor->rate_hz = 0U;
      motor->current_rate_hz = 0U;
      motor->move_active = 0U;
    }
    else
    {
      motor->move_active = 0U;
    }
    stepper_apply_outputs(motor);
  }
}

static void stepper_set_direction(uint8_t motor_id, uint8_t direction)
{
  uint8_t idx;

  if (motor_id == 0U)
  {
    for (idx = 0U; idx < STEPPER_COUNT; idx++)
    {
      motors[idx].direction = (direction != 0U) ? 1U : 0U;
      stepper_apply_outputs(&motors[idx]);
    }
    return;
  }

  stepper_motor_t *motor = get_motor_by_id(motor_id);
  if (motor != NULL)
  {
    motor->direction = (direction != 0U) ? 1U : 0U;
    stepper_apply_outputs(motor);
  }
}

static void stepper_set_step_rate_hz(uint8_t motor_id, uint32_t rate_hz)
{
  uint32_t period_ticks;
  uint32_t arr;
  uint32_t pulse;
  uint8_t idx;

  if (rate_hz > STEPPER_MAX_RATE_HZ)
  {
    rate_hz = STEPPER_MAX_RATE_HZ;
  }

  if (motor_id == 0U)
  {
    for (idx = 0U; idx < STEPPER_COUNT; idx++)
    {
      motors[idx].rate_hz = rate_hz;
      if (rate_hz == 0U)
      {
        stepper_stop_pwm(&motors[idx]);
      }
      else if (motors[idx].enabled != 0U)
      {
        period_ticks = STEPPER_TIMER_CLK_HZ / rate_hz;
        if (period_ticks < 2U)
        {
          period_ticks = 2U;
        }
        arr = period_ticks - 1U;
        pulse = period_ticks / 2U;
        __HAL_TIM_SET_AUTORELOAD(motors[idx].htim, arr);
        __HAL_TIM_SET_COMPARE(motors[idx].htim, motors[idx].tim_channel, pulse);
        __HAL_TIM_SET_COUNTER(motors[idx].htim, 0U);
        stepper_start_pwm(&motors[idx]);
      }
    }
    return;
  }

  stepper_motor_t *motor = get_motor_by_id(motor_id);
  if (motor != NULL)
  {
    motor->rate_hz = rate_hz;

    if (rate_hz == 0U)
    {
      stepper_stop_pwm(motor);
      return;
    }

    period_ticks = STEPPER_TIMER_CLK_HZ / rate_hz;
    if (period_ticks < 2U)
    {
      period_ticks = 2U;
    }

    arr = period_ticks - 1U;
    pulse = period_ticks / 2U;

    __HAL_TIM_SET_AUTORELOAD(motor->htim, arr);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->tim_channel, pulse);
    __HAL_TIM_SET_COUNTER(motor->htim, 0U);

    if (motor->enabled != 0U)
    {
      stepper_start_pwm(motor);
    }
  }
}

static void stepper_set_move_rate_hz(uint8_t motor_id, uint32_t rate_hz)
{
  uint8_t idx;

  if (rate_hz > STEPPER_MAX_RATE_HZ)
  {
    rate_hz = STEPPER_MAX_RATE_HZ;
  }

  if (motor_id == 0U)
  {
    for (idx = 0U; idx < STEPPER_COUNT; idx++)
    {
      motors[idx].move_rate_hz = rate_hz;
    }
    return;
  }

  stepper_motor_t *motor = get_motor_by_id(motor_id);
  if (motor != NULL)
  {
    motor->move_rate_hz = rate_hz;
  }
}

static void stepper_update_ramp(stepper_motor_t *motor)
{
  uint32_t now = HAL_GetTick();
  uint32_t remaining_steps;
  uint32_t next_rate = motor->current_rate_hz;

  if ((now - motor->ramp_last_tick) < STEPPER_RAMP_INTERVAL_MS)
  {
    return;
  }
  motor->ramp_last_tick = now;

  remaining_steps = motor->move_target_steps - motor->move_done_steps;
  if (remaining_steps <= STEPPER_DECEL_STEPS)
  {
    if (next_rate > STEPPER_START_RATE_HZ)
    {
      next_rate = (next_rate > (STEPPER_START_RATE_HZ + STEPPER_RAMP_STEP_HZ)) ?
                  (next_rate - STEPPER_RAMP_STEP_HZ) : STEPPER_START_RATE_HZ;
    }
  }
  else if (next_rate < motor->move_rate_hz)
  {
    next_rate += STEPPER_RAMP_STEP_HZ;
    if (next_rate > motor->move_rate_hz)
    {
      next_rate = motor->move_rate_hz;
    }
  }

  if (next_rate != motor->current_rate_hz)
  {
    motor->current_rate_hz = next_rate;
    stepper_set_step_rate_hz((uint8_t)((motor - motors) + 1), next_rate);
  }
}

static void stepper_init_all(void)
{
  uint8_t idx;

  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    motors[idx].enabled = 0U;
    motors[idx].direction = 0U;
    motors[idx].rate_hz = STEPPER_DEFAULT_RATE_HZ;
    motors[idx].move_target_steps = 0U;
    motors[idx].move_done_steps = 0U;
    motors[idx].move_active = 0U;
    stepper_apply_outputs(&motors[idx]);
  }
}

static void send_text_line(const char *text)
{
  uint16_t packet_len = build_text_packet(tx_text_packet, text);

  if (HAL_UART_Transmit_IT(&huart2, tx_text_packet, packet_len) == HAL_OK)
  {
    uart_tx_ready = 0;
  }
}

/* Messages are queued so they never race the ACK/NACK reply for the same command. */
static void arm_post_message(const char *text)
{
  if (arm_message_pending == 0U)
  {
    (void)snprintf(arm_message, sizeof(arm_message), "%s", text);
    arm_message_pending = 1U;
  }
}

static void arm_post_throttled(const char *text)
{
  uint32_t now = HAL_GetTick();

  if (now >= next_arm_message_tick)
  {
    next_arm_message_tick = now + 1000U;
    arm_post_message(text);
  }
}

static void arm_flush_message(void)
{
  if ((arm_message_pending != 0U) && (uart_tx_ready != 0U))
  {
    send_text_line(arm_message);
    arm_message_pending = 0U;
  }
}

static float joint_deg_to_raw(uint8_t idx, float deg)
{
  float steps_per_deg = ((float)STEPPER_STEPS_PER_REV * ARM_MICROSTEP * joint_gear_ratio[idx]) / 360.0f;

  return deg * steps_per_deg * joint_dir_sign[idx];
}

static float joint_raw_to_deg(uint8_t idx, float raw)
{
  float steps_per_deg = ((float)STEPPER_STEPS_PER_REV * ARM_MICROSTEP * joint_gear_ratio[idx]) / 360.0f;

  return raw / (steps_per_deg * joint_dir_sign[idx]);
}

static void joints_to_array(const arm_joints_t *joints, float out[STEPPER_COUNT])
{
  out[0] = joints->base;
  out[1] = joints->shoulder;
  out[2] = joints->elbow;
  out[3] = joints->wrist;
}

static arm_joints_t arm_measured_joints(void)
{
  arm_joints_t joints;

  joints.base = joint_raw_to_deg(0U, (float)motors[0].position_steps);
  joints.shoulder = joint_raw_to_deg(1U, (float)motors[1].position_steps);
  joints.elbow = joint_raw_to_deg(2U, (float)motors[2].position_steps);
  joints.wrist = joint_raw_to_deg(3U, (float)motors[3].position_steps);
  return joints;
}

static uint8_t arm_pointer_horizontal(const arm_joints_t *joints)
{
  float pitch = joints->shoulder + joints->elbow + joints->wrist;

  return (fabsf(pitch - ARM_POINTER_PITCH_DEG) <= ARM_POINTER_TOLERANCE_DEG) ? 1U : 0U;
}

static uint8_t arm_motors_enabled(void)
{
  uint8_t idx;

  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    if (motors[idx].enabled == 0U)
    {
      return 0U;
    }
  }
  return 1U;
}

static uint8_t motor_is_busy(const stepper_motor_t *motor)
{
  return *(const volatile uint8_t *)&motor->move_active;
}

static float approach(float current, float target, float max_change)
{
  if (target > (current + max_change))
  {
    return current + max_change;
  }
  if (target < (current - max_change))
  {
    return current - max_change;
  }
  return target;
}

static void arm_halt(void)
{
  uint8_t idx;

  stepper_set_step_rate_hz(0U, 0U);
  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    motors[idx].move_active = 0U;
    motors[idx].move_ramp = 1U;
    joint_target_raw[idx] = (float)motors[idx].position_steps;
  }
  tip_velocity[0] = 0.0f;
  tip_velocity[1] = 0.0f;
  tip_velocity[2] = 0.0f;
  arm_mode = ARM_MODE_IDLE;
  arm_ready_pending = 0U;
}

static void arm_set_home(void)
{
  uint8_t idx;

  arm_halt();
  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    motors[idx].position_steps = 0;
    joint_target_raw[idx] = 0.0f;
    ik_offset_raw[idx] = 0.0f;
  }
  base_target_deg = 0.0f;
  arm_homed = 1U;
}

/* Starts a burst of whole steps that lasts about one tick (rate = steps / tick). */
static void arm_start_burst(uint8_t idx, int32_t steps)
{
  stepper_motor_t *motor = &motors[idx];
  uint8_t direction = (steps > 0) ? 1U : 0U;
  uint32_t count = (steps > 0) ? (uint32_t)steps : (uint32_t)(-steps);
  uint32_t rate;

  if (count > ARM_MAX_BURST_STEPS)
  {
    count = ARM_MAX_BURST_STEPS;
  }
  /* Target completing the burst in ~15 ms so it is guaranteed finished before the next 20 ms tick. */
  rate = (count * 1000U) / 15U;
  if (rate < 60U)
  {
    rate = 60U;
  }
  if (rate > STEPPER_MAX_RATE_HZ)
  {
    rate = STEPPER_MAX_RATE_HZ;
  }

  if (motor->direction != direction)
  {
    stepper_set_direction((uint8_t)(idx + 1U), direction);
  }
  motor->move_target_steps = count;
  motor->move_done_steps = 0U;
  motor->move_ramp = 0U;
  motor->move_rate_hz = rate;
  motor->current_rate_hz = rate;
  motor->move_active = 1U;
  stepper_set_step_rate_hz((uint8_t)(idx + 1U), rate);
}

static void arm_issue_bursts(void)
{
  uint8_t idx;
  float error;

  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    if (motor_is_busy(&motors[idx]) != 0U)
    {
      continue;
    }

    error = joint_target_raw[idx] - (float)motors[idx].position_steps;
    if (fabsf(error) >= ARM_STEP_THRESHOLD)
    {
      arm_start_burst(idx, lroundf(error));
    }
  }
}

static float arm_max_lag_steps(void)
{
  uint8_t idx;
  float lag;
  float max_lag = 0.0f;

  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    lag = fabsf(joint_target_raw[idx] - (float)motors[idx].position_steps);
    if (lag > max_lag)
    {
      max_lag = lag;
    }
  }
  return max_lag;
}

/* Captures the real pose as the starting point of Cartesian control. Returns an error text or NULL. */
static const char *arm_begin_cartesian(void)
{
  arm_joints_t measured;
  arm_joints_t solved;
  arm_ik_status_t status;
  float solved_deg[STEPPER_COUNT];
  uint8_t idx;

  arm_halt();
  if (arm_homed == 0U)
  {
    return "set HOME first";
  }
  if (arm_motors_enabled() == 0U)
  {
    return "enable all motors first";
  }

  measured = arm_measured_joints();
  if (arm_pointer_horizontal(&measured) == 0U)
  {
    return "pointer not horizontal, run READY";
  }

  tip_target = arm_forward(&measured);
  status = arm_inverse(&tip_target, measured.base, &solved);
  if (status != ARM_IK_OK)
  {
    return arm_ik_status_text(status);
  }

  /* Any small difference between the real joints and the ideal solution is kept as a constant offset. */
  joints_to_array(&solved, solved_deg);
  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    ik_offset_raw[idx] = (float)motors[idx].position_steps - joint_deg_to_raw(idx, solved_deg[idx]);
    joint_target_raw[idx] = (float)motors[idx].position_steps;
  }
  base_target_deg = solved.base;
  return NULL;
}

/* Moves the tip target toward desired (limited by joint speed). Returns 1 if the target advanced. */
static uint8_t arm_try_tip_move(const arm_point_t *desired, float dt)
{
  arm_point_t candidate = *desired;
  arm_joints_t solved;
  arm_ik_status_t status;
  float solved_deg[STEPPER_COUNT];
  float raw[STEPPER_COUNT];
  float max_delta;
  float delta;
  float total_scale = 1.0f;
  const float max_step = ARM_MAX_JOINT_RATE_HZ * dt;
  uint8_t pass;
  uint8_t idx;

  for (pass = 0U; pass < 4U; pass++)
  {
    status = arm_inverse(&candidate, base_target_deg, &solved);
    if (status != ARM_IK_OK)
    {
      arm_post_throttled(arm_ik_status_text(status));
      return 0U;
    }

    joints_to_array(&solved, solved_deg);
    max_delta = 0.0f;
    for (idx = 0U; idx < STEPPER_COUNT; idx++)
    {
      raw[idx] = joint_deg_to_raw(idx, solved_deg[idx]) + ik_offset_raw[idx];
      delta = fabsf(raw[idx] - joint_target_raw[idx]);
      if (delta > max_delta)
      {
        max_delta = delta;
      }
    }

    if (max_delta <= max_step)
    {
      tip_target = candidate;
      base_target_deg = solved.base;
      for (idx = 0U; idx < STEPPER_COUNT; idx++)
      {
        joint_target_raw[idx] = raw[idx];
      }
      return 1U;
    }

    /* Too fast for one joint: shorten the move so every joint slows down together. */
    total_scale *= (max_step / max_delta) * 0.95f;
    candidate.x = tip_target.x + ((desired->x - tip_target.x) * total_scale);
    candidate.y = tip_target.y + ((desired->y - tip_target.y) * total_scale);
    candidate.z = tip_target.z + ((desired->z - tip_target.z) * total_scale);
  }

  return 0U;
}

static void arm_read_joystick(float axis[JOYSTICK_AXIS_COUNT])
{
  uint8_t idx;
  uint16_t raw;
  int32_t offset;
  uint32_t magnitude;
  float shaped;
  float rx;
  float ry;

  for (idx = 0U; idx < JOYSTICK_AXIS_COUNT; idx++)
  {
    raw = joystick_read(joystick_channels[idx]);
    joystick_raw_values[idx] = raw;
    joystick_filtered_values[idx] = (uint16_t)((3U * joystick_filtered_values[idx] + raw) / 4U);

    offset = (int32_t)joystick_filtered_values[idx] - (int32_t)joystick_center_values[idx];
    magnitude = (offset < 0) ? (uint32_t)(-offset) : (uint32_t)offset;
    if (magnitude <= JOYSTICK_DEADZONE)
    {
      axis[idx] = 0.0f;
      continue;
    }

    shaped = (float)(magnitude - JOYSTICK_DEADZONE) / (float)(2048U - JOYSTICK_DEADZONE - 100U);
    if (shaped > 1.0f)
    {
      shaped = 1.0f;
    }
    /* Blend linear and quadratic (65% linear, 35% quadratic) so small stick deflections
       react promptly rather than feeling dead, while keeping good fine control. */
    shaped = (0.65f * shaped) + (0.35f * shaped * shaped);
    axis[idx] = ((offset < 0) ? -shaped : shaped) * joystick_sign[idx];
  }

  /* Cross-talk suppression on Right Stick (X=Base, Y=Reach):
     If the user is pushing primarily along one axis, suppress accidental diagonal bleed
     from the perpendicular axis so the base turns cleanly without unexpected Y drift. */
  rx = fabsf(axis[JOY_RIGHT_X]);
  ry = fabsf(axis[JOY_RIGHT_Y]);
  if ((rx > 0.0f) || (ry > 0.0f))
  {
    if (rx > (1.8f * ry))
    {
      axis[JOY_RIGHT_Y] = 0.0f; /* Pure X / Base sweep */
    }
    else if (ry > (1.8f * rx))
    {
      axis[JOY_RIGHT_X] = 0.0f; /* Pure Y / Reach */
    }
  }
}

static void arm_tick_joystick(float dt, uint8_t lagging)
{
  float axis[JOYSTICK_AXIS_COUNT];
  float desired[3];
  float magnitude;
  float max_change = ARM_TIP_ACCEL_MM_S2 * dt;
  arm_point_t next;
  uint8_t idx;

  arm_read_joystick(axis);

  desired[0] = axis[JOY_RIGHT_X] * arm_speed_mm_s;
  desired[1] = axis[JOY_RIGHT_Y] * arm_speed_mm_s;
  desired[2] = axis[JOY_LEFT_Y] * arm_speed_mm_s;
  magnitude = sqrtf((desired[0] * desired[0]) + (desired[1] * desired[1]) + (desired[2] * desired[2]));
  if (magnitude > arm_speed_mm_s)
  {
    for (idx = 0U; idx < 3U; idx++)
    {
      desired[idx] *= arm_speed_mm_s / magnitude;
    }
  }

  for (idx = 0U; idx < 3U; idx++)
  {
    tip_velocity[idx] = approach(tip_velocity[idx], desired[idx], max_change);
  }

  if ((lagging != 0U) ||
      ((fabsf(tip_velocity[0]) < 0.01f) && (fabsf(tip_velocity[1]) < 0.01f) && (fabsf(tip_velocity[2]) < 0.01f)))
  {
    return;
  }

  next.x = tip_target.x + (tip_velocity[0] * dt);
  next.y = tip_target.y + (tip_velocity[1] * dt);
  next.z = tip_target.z + (tip_velocity[2] * dt);
  if (arm_try_tip_move(&next, dt) == 0U)
  {
    tip_velocity[0] = 0.0f;
    tip_velocity[1] = 0.0f;
    tip_velocity[2] = 0.0f;
  }
}

static void arm_tick_goto(float dt, uint8_t lagging)
{
  float dx = goto_goal.x - tip_target.x;
  float dy = goto_goal.y - tip_target.y;
  float dz = goto_goal.z - tip_target.z;
  float distance = sqrtf((dx * dx) + (dy * dy) + (dz * dz));
  float target_speed;
  float step;
  arm_point_t next;

  if (distance < 0.05f)
  {
    arm_mode = ARM_MODE_SETTLE;
    return;
  }
  if (lagging != 0U)
  {
    return;
  }

  target_speed = sqrtf(2.0f * ARM_TIP_ACCEL_MM_S2 * distance);
  if (target_speed > arm_speed_mm_s)
  {
    target_speed = arm_speed_mm_s;
  }
  if (target_speed < 1.0f)
  {
    target_speed = 1.0f;
  }
  goto_speed = approach(goto_speed, target_speed, ARM_TIP_ACCEL_MM_S2 * dt);

  step = goto_speed * dt;
  if (step >= distance)
  {
    next = goto_goal;
  }
  else
  {
    next.x = tip_target.x + (dx / distance * step);
    next.y = tip_target.y + (dy / distance * step);
    next.z = tip_target.z + (dz / distance * step);
  }

  if (arm_try_tip_move(&next, dt) == 0U)
  {
    arm_halt();
  }
}

static void arm_tick_jmove(float dt, uint8_t lagging)
{
  float remaining = jmove_distance - jmove_progress;
  float target_speed;
  float fraction;
  uint8_t idx;

  if (lagging != 0U)
  {
    return;
  }

  target_speed = sqrtf(2.0f * ARM_JMOVE_ACCEL_HZ_S * remaining);
  if (target_speed > ARM_JMOVE_MAX_RATE_HZ)
  {
    target_speed = ARM_JMOVE_MAX_RATE_HZ;
  }
  if (target_speed < ARM_JMOVE_MIN_RATE_HZ)
  {
    target_speed = ARM_JMOVE_MIN_RATE_HZ;
  }
  jmove_speed = approach(jmove_speed, target_speed, ARM_JMOVE_ACCEL_HZ_S * dt);

  jmove_progress += jmove_speed * dt;
  if (jmove_progress >= jmove_distance)
  {
    jmove_progress = jmove_distance;
    arm_mode = ARM_MODE_SETTLE;
  }

  fraction = jmove_progress / jmove_distance;
  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    joint_target_raw[idx] = jmove_start_raw[idx] + ((jmove_goal_raw[idx] - jmove_start_raw[idx]) * fraction);
  }
}

static void arm_tick(void)
{
  uint32_t now = HAL_GetTick();
  float dt = (float)ARM_TICK_MS / 1000.0f;
  uint8_t lagging;

  if ((arm_mode == ARM_MODE_IDLE) || (now < next_arm_tick))
  {
    return;
  }
  next_arm_tick = now + ARM_TICK_MS;

  if (arm_motors_enabled() == 0U)
  {
    arm_halt();
    arm_post_message("motors disabled, arm stopped");
    return;
  }

  lagging = (arm_max_lag_steps() > ARM_MAX_LAG_STEPS) ? 1U : 0U;

  switch (arm_mode)
  {
    case ARM_MODE_JOYSTICK:
      arm_tick_joystick(dt, lagging);
      break;

    case ARM_MODE_GOTO:
      arm_tick_goto(dt, lagging);
      break;

    case ARM_MODE_JMOVE:
      arm_tick_jmove(dt, lagging);
      break;

    case ARM_MODE_SETTLE:
      if ((arm_max_lag_steps() < ARM_STEP_THRESHOLD) &&
          (motor_is_busy(&motors[0]) == 0U) && (motor_is_busy(&motors[1]) == 0U) &&
          (motor_is_busy(&motors[2]) == 0U) && (motor_is_busy(&motors[3]) == 0U))
      {
        arm_mode = ARM_MODE_IDLE;
        arm_ready_pending = 0U;
        tip_velocity[0] = 0.0f;
        tip_velocity[1] = 0.0f;
        tip_velocity[2] = 0.0f;
        arm_post_message("arm motion complete");
      }
      break;

    default:
      break;
  }

  arm_issue_bursts();
}

static const char *arm_start_jmove(const arm_joints_t *goal, uint8_t is_ready_move)
{
  arm_joints_t start;
  arm_joints_t sample;
  arm_ik_status_t status;
  float goal_deg[STEPPER_COUNT];
  float start_deg[STEPPER_COUNT];
  float delta;
  float fraction;
  uint8_t idx;
  uint8_t pass;

  arm_halt();
  if (arm_homed == 0U)
  {
    return "set HOME first";
  }
  if (arm_motors_enabled() == 0U)
  {
    return "enable all motors first";
  }

  status = arm_check_joints(goal);
  if (status != ARM_IK_OK)
  {
    return arm_ik_status_text(status);
  }

  start = arm_measured_joints();
  joints_to_array(&start, start_deg);
  joints_to_array(goal, goal_deg);

  /* The joints move in a straight line in joint space, so check the whole path. */
  for (pass = 1U; pass <= 20U; pass++)
  {
    fraction = (float)pass / 20.0f;
    sample.base = start.base + ((goal->base - start.base) * fraction);
    sample.shoulder = start.shoulder + ((goal->shoulder - start.shoulder) * fraction);
    sample.elbow = start.elbow + ((goal->elbow - start.elbow) * fraction);
    sample.wrist = start.wrist + ((goal->wrist - start.wrist) * fraction);
    status = arm_check_joints(&sample);
    if (status != ARM_IK_OK)
    {
      return "path leaves safe area";
    }
  }

  jmove_distance = 0.0f;
  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    jmove_start_raw[idx] = (float)motors[idx].position_steps;
    jmove_goal_raw[idx] = roundf(joint_deg_to_raw(idx, goal_deg[idx]));
    delta = fabsf(jmove_goal_raw[idx] - jmove_start_raw[idx]);
    if (delta > jmove_distance)
    {
      jmove_distance = delta;
    }
  }

  if (jmove_distance < 1.0f)
  {
    return NULL;
  }

  jmove_progress = 0.0f;
  jmove_speed = ARM_JMOVE_MIN_RATE_HZ;
  arm_ready_pending = is_ready_move;
  arm_mode = ARM_MODE_JMOVE;
  next_arm_tick = HAL_GetTick();
  return NULL;
}

/* Joint-space move to the pose where the pointer is horizontal at the ready position. */
static const char *arm_start_ready(void)
{
  arm_point_t ready = {ARM_READY_X_MM, ARM_READY_Y_MM, ARM_READY_Z_MM};
  arm_joints_t solved;
  arm_ik_status_t status = arm_inverse(&ready, 0.0f, &solved);

  if (status != ARM_IK_OK)
  {
    return arm_ik_status_text(status);
  }

  /* Keep the current base angle, only the shoulder/elbow/wrist move to the ready pose. */
  solved.base = arm_measured_joints().base;
  return arm_start_jmove(&solved, 1U);
}

static const char *arm_start_goto(const arm_point_t *goal)
{
  arm_joints_t solved;
  arm_ik_status_t status;
  const char *error = arm_begin_cartesian();

  if (error != NULL)
  {
    return error;
  }

  status = arm_inverse(goal, base_target_deg, &solved);
  if (status != ARM_IK_OK)
  {
    return arm_ik_status_text(status);
  }

  goto_goal = *goal;
  goto_speed = 0.0f;
  arm_mode = ARM_MODE_GOTO;
  next_arm_tick = HAL_GetTick();
  return NULL;
}

static const char *arm_set_joystick(uint8_t enabled)
{
  uint8_t idx;
  uint16_t center;
  const char *error;

  if (enabled == 0U)
  {
    if (arm_mode == ARM_MODE_JOYSTICK)
    {
      tip_velocity[0] = 0.0f;
      tip_velocity[1] = 0.0f;
      tip_velocity[2] = 0.0f;
      arm_mode = ARM_MODE_SETTLE;
    }
    return NULL;
  }

  error = arm_begin_cartesian();
  if (error != NULL)
  {
    return error;
  }

  /* The sticks must be released (centered) while the center is measured. */
  for (idx = 0U; idx < JOYSTICK_AXIS_COUNT; idx++)
  {
    center = joystick_calibrate_center(joystick_channels[idx]);
    if ((center < 1200U) || (center > 2900U))
    {
      return "joystick not centered or not connected";
    }
    joystick_center_values[idx] = center;
    joystick_raw_values[idx] = center;
    joystick_filtered_values[idx] = center;
  }

  arm_mode = ARM_MODE_JOYSTICK;
  next_arm_tick = HAL_GetTick();
  return NULL;
}

static uint8_t parse_float_list(const char *text, float *values, uint8_t count)
{
  char *end = NULL;
  uint8_t idx;

  for (idx = 0U; idx < count; idx++)
  {
    values[idx] = strtof(text, &end);
    if (end == text)
    {
      return 0U;
    }
    text = end;
    if ((idx + 1U) < count)
    {
      if (*text != ',')
      {
        return 0U;
      }
      text++;
    }
  }
  return (*text == '\0') ? 1U : 0U;
}

static void send_pose_text(void)
{
  char status[TX_TEXT_BUFFER_SIZE + 1U];
  arm_joints_t joints = arm_measured_joints();
  arm_point_t tip = arm_forward(&joints);

  (void)snprintf(status,
                 sizeof(status),
                 "POSE H%u M%u P%u X%ld Y%ld Z%ld | B%ld S%ld E%ld W%ld",
                 (unsigned int)arm_homed,
                 (unsigned int)arm_mode,
                 (unsigned int)arm_pointer_horizontal(&joints),
                 lroundf(tip.x * 10.0f),
                 lroundf(tip.y * 10.0f),
                 lroundf(tip.z * 10.0f),
                 lroundf(joints.base * 10.0f),
                 lroundf(joints.shoulder * 10.0f),
                 lroundf(joints.elbow * 10.0f),
                 lroundf(joints.wrist * 10.0f));
  send_text_line(status);
}

static void send_status_text(void)
{
  uint16_t packet_len;
  char status[TX_TEXT_BUFFER_SIZE + 1U];

  (void)snprintf(status,
                 sizeof(status),
                 "M1 E%u D%u S%lu B%u R%lu | M2 E%u D%u S%lu B%u R%lu | M3 E%u D%u S%lu B%u R%lu | M4 E%u D%u S%lu B%u R%lu",
                 (unsigned int)motors[0].enabled,
                 (unsigned int)motors[0].direction,
                 (unsigned long)motors[0].rate_hz,
                 (unsigned int)motors[0].move_active,
                 (unsigned long)(motors[0].move_target_steps - motors[0].move_done_steps),
                 (unsigned int)motors[1].enabled,
                 (unsigned int)motors[1].direction,
                 (unsigned long)motors[1].rate_hz,
                 (unsigned int)motors[1].move_active,
                 (unsigned long)(motors[1].move_target_steps - motors[1].move_done_steps),
                 (unsigned int)motors[2].enabled,
                 (unsigned int)motors[2].direction,
                 (unsigned long)motors[2].rate_hz,
                 (unsigned int)motors[2].move_active,
                 (unsigned long)(motors[2].move_target_steps - motors[2].move_done_steps),
                 (unsigned int)motors[3].enabled,
                 (unsigned int)motors[3].direction,
                 (unsigned long)motors[3].rate_hz,
                 (unsigned int)motors[3].move_active,
                 (unsigned long)(motors[3].move_target_steps - motors[3].move_done_steps));

  packet_len = build_text_packet(tx_text_packet, status);

  if (HAL_UART_Transmit_IT(&huart2, tx_text_packet, packet_len) == HAL_OK)
  {
    uart_tx_ready = 0;
  }
}

static void send_joystick_status(void)
{
  uint16_t packet_len;
  char status[TX_TEXT_BUFFER_SIZE + 1U];

  (void)snprintf(status,
                 sizeof(status),
                 "JOY %u RAW LX%u LY%u RX%u RY%u CTR %u %u %u %u",
                 (unsigned int)(arm_mode == ARM_MODE_JOYSTICK),
                 (unsigned int)joystick_raw_values[0],
                 (unsigned int)joystick_raw_values[1],
                 (unsigned int)joystick_raw_values[2],
                 (unsigned int)joystick_raw_values[3],
                 (unsigned int)joystick_center_values[0],
                 (unsigned int)joystick_center_values[1],
                 (unsigned int)joystick_center_values[2],
                 (unsigned int)joystick_center_values[3]);
  packet_len = build_text_packet(tx_text_packet, status);
  if (HAL_UART_Transmit_IT(&huart2, tx_text_packet, packet_len) == HAL_OK)
  {
    uart_tx_ready = 0;
  }
}

static uint8_t arm_command_result(const char *error)
{
  if (error != NULL)
  {
    arm_post_message(error);
    return 0U;
  }
  return 1U;
}

static uint8_t parse_ascii_command(const uint8_t *payload, uint16_t len)
{
  uint8_t ok = 0U;
  char cmd[TX_TEXT_BUFFER_SIZE + 1U];
  char *eq;
  uint8_t motor_id = 1U;
  char *cmd_name;

  if (len > TX_TEXT_BUFFER_SIZE)
  {
    return 0U;
  }

  memcpy(cmd, payload, len);
  cmd[len] = '\0';

  if (strcmp(cmd, "JOY=1") == 0)
  {
    return arm_command_result(arm_set_joystick(1U));
  }
  if (strcmp(cmd, "JOY=0") == 0)
  {
    return arm_command_result(arm_set_joystick(0U));
  }
  if (strcmp(cmd, "JOY_STATUS") == 0)
  {
    send_joystick_status();
    return 1U;
  }
  if (strcmp(cmd, "HOME") == 0)
  {
    arm_set_home();
    return 1U;
  }
  if (strcmp(cmd, "READY") == 0)
  {
    return arm_command_result(arm_start_ready());
  }
  if (strcmp(cmd, "POSE") == 0)
  {
    send_pose_text();
    return 1U;
  }

  if ((strcmp(cmd, "STOP") == 0) || (strcmp(cmd, "STOPALL") == 0))
  {
    arm_halt();
    return 1U;
  }

  eq = strchr(cmd, '=');
  if (eq == NULL)
  {
    return 0U;
  }

  *eq = '\0';
  cmd_name = cmd;

  if (strcmp(cmd, "GOTO") == 0)
  {
    float values[3];
    arm_point_t goal;

    if (parse_float_list(eq + 1, values, 3U) == 0U)
    {
      return 0U;
    }
    goal.x = values[0];
    goal.y = values[1];
    goal.z = values[2];
    return arm_command_result(arm_start_goto(&goal));
  }
  if (strcmp(cmd, "JMOVE") == 0)
  {
    float values[4];
    arm_joints_t goal;

    if (parse_float_list(eq + 1, values, 4U) == 0U)
    {
      return 0U;
    }
    goal.base = values[0];
    goal.shoulder = values[1];
    goal.elbow = values[2];
    goal.wrist = values[3];
    return arm_command_result(arm_start_jmove(&goal, 0U));
  }
  if (strcmp(cmd, "SPEED") == 0)
  {
    float values[1];

    if ((parse_float_list(eq + 1, values, 1U) == 0U) || (values[0] < 1.0f) || (values[0] > ARM_SPEED_MAX_MM_S))
    {
      return 0U;
    }
    arm_speed_mm_s = values[0];
    return 1U;
  }
  if (strcmp(cmd, "JOYSIGN") == 0)
  {
    float values[JOYSTICK_AXIS_COUNT];
    uint8_t idx;

    if (parse_float_list(eq + 1, values, JOYSTICK_AXIS_COUNT) == 0U)
    {
      return 0U;
    }
    for (idx = 0U; idx < JOYSTICK_AXIS_COUNT; idx++)
    {
      joystick_sign[idx] = (values[idx] < 0.0f) ? -1.0f : 1.0f;
    }
    return 1U;
  }

  /* Anything below drives a motor directly, so it cancels any arm motion in progress. */
  if (arm_mode != ARM_MODE_IDLE)
  {
    arm_halt();
  }
  if (strncmp(cmd, "M", 1) == 0)
  {
    if ((cmd[1] >= '1') && (cmd[1] <= '4') && (cmd[2] == '_'))
    {
      motor_id = (uint8_t)(cmd[1] - '0');
      cmd_name = &cmd[3];
    }
    else
    {
      return 0U;
    }
  }
  else if (strncmp(cmd, "ALL_", 4) == 0)
  {
    motor_id = 0U;
    cmd_name = &cmd[4];
  }

  if ((strcmp(cmd_name, "ENA") == 0) || (strcmp(cmd_name, "EN") == 0))
  {
    if (strcmp(eq + 1, "1") == 0)
    {
      stepper_set_enable(motor_id, 1U);
      ok = 1U;
    }
    else if (strcmp(eq + 1, "0") == 0)
    {
      stepper_set_enable(motor_id, 0U);
      ok = 1U;
    }
  }
  else if (strcmp(cmd_name, "DIR") == 0)
  {
    if (strcmp(eq + 1, "1") == 0)
    {
      stepper_set_direction(motor_id, 1U);
      ok = 1U;
    }
    else if (strcmp(eq + 1, "0") == 0)
    {
      stepper_set_direction(motor_id, 0U);
      ok = 1U;
    }
  }
  else if ((strcmp(cmd_name, "STEP") == 0) || (strcmp(cmd_name, "SPD") == 0))
  {
    char *end_ptr = NULL;
    unsigned long raw = strtoul(eq + 1, &end_ptr, 10);

    if (end_ptr != (eq + 1) && *end_ptr == '\0')
    {
      stepper_set_step_rate_hz(motor_id, (uint32_t)raw);
      ok = 1U;
    }
  }
  else if (strcmp(cmd_name, "RATE") == 0)
  {
    char *end_ptr = NULL;
    unsigned long raw = strtoul(eq + 1, &end_ptr, 10);

    if (end_ptr != (eq + 1) && *end_ptr == '\0')
    {
      stepper_set_move_rate_hz(motor_id, (uint32_t)raw);
      ok = (motor_id <= STEPPER_COUNT) || (motor_id == 0U);
    }
  }
  else if ((strcmp(cmd_name, "DEG") == 0) || (strcmp(cmd_name, "MOVE") == 0))
  {
    char *end_ptr = NULL;
    unsigned long raw = strtoul(eq + 1, &end_ptr, 10);

    if (end_ptr != (eq + 1) && *end_ptr == '\0')
    {
      ok = stepper_move_degrees(motor_id, (uint32_t)raw);
    }
  }

  return ok;
}

static uint8_t parse_binary_command(const uint8_t *payload, uint16_t len)
{
  uint8_t motor_id;

  if (len < 2U)
  {
    return 0U;
  }

  if (arm_mode != ARM_MODE_IDLE)
  {
    arm_halt();
  }

  motor_id = 1U;

  switch (payload[0])
  {
    case 0x01U:
      if (len >= 3U)
      {
        motor_id = payload[1];
        stepper_set_enable(motor_id, payload[2]);
        return (motor_id <= STEPPER_COUNT) || (motor_id == 0U);
      }
      stepper_set_enable(1U, payload[1]);
      return 1U;

    case 0x02U:
      if (len >= 3U)
      {
        motor_id = payload[1];
        stepper_set_direction(motor_id, payload[2]);
        return (motor_id <= STEPPER_COUNT) || (motor_id == 0U);
      }
      stepper_set_direction(1U, payload[1]);
      return 1U;

    case 0x03U:
      if (len >= 4U)
      {
        uint32_t rate_hz = (uint32_t)payload[2] | ((uint32_t)payload[3] << 8);
        motor_id = payload[1];
        stepper_set_step_rate_hz(motor_id, rate_hz);
        return (motor_id <= STEPPER_COUNT) || (motor_id == 0U);
      }
      if (len >= 3U)
      {
        uint32_t rate_hz = (uint32_t)payload[1] | ((uint32_t)payload[2] << 8);
        stepper_set_step_rate_hz(1U, rate_hz);
        return 1U;
      }
      break;

    case 0x04U:
      if (len >= 4U)
      {
        uint32_t degrees = (uint32_t)payload[2] | ((uint32_t)payload[3] << 8);
        motor_id = payload[1];
        if ((motor_id <= STEPPER_COUNT) || (motor_id == 0U))
        {
          return stepper_move_degrees(motor_id, degrees);
        }
      }
      break;

    default:
      break;
  }

  return 0U;
}

static void process_rx_frame(void)
{
  uint8_t valid = 0U;
  uint8_t pkt_type;
  uint16_t pkt_len;
  const uint8_t *payload;

  pkt_type = rx_buffer[2];
  pkt_len = (uint16_t)rx_buffer[3] | ((uint16_t)rx_buffer[4] << 8);

  if ((rx_index != (uint16_t)(5U + pkt_len)) || (pkt_len > (RX_BUFFER_MAX - 5U)))
  {
    send_ack(PKT_TYPE_NACK);
    return;
  }

  payload = &rx_buffer[5];

  if (pkt_type == PKT_TYPE_TEXT)
  {
    valid = parse_ascii_command(payload, pkt_len);
  }
  else if (pkt_type == PKT_TYPE_BIN_CMD)
  {
    valid = parse_binary_command(payload, pkt_len);
  }

  send_ack((valid != 0U) ? PKT_TYPE_ACK : PKT_TYPE_NACK);
  if (valid != 0U)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  joystick_init();
  stepper_init_all();

  next_status_text_tick = HAL_GetTick();
  start_uart_rx_it();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    arm_flush_message();

    if (uart_tx_ready && (HAL_GetTick() >= next_status_text_tick))
    {
      if (pose_status_turn == 0U)
      {
        send_status_text();
      }
      else
      {
        send_pose_text();
      }
      pose_status_turn ^= 1U;
      next_status_text_tick = HAL_GetTick() + ARM_STATUS_PERIOD_MS;
    }

    if (rx_packet_ready)
    {
      process_rx_frame();
      rx_packet_ready = 0;
      rx_index = 0U;
      rx_frame_len = 0U;
      start_uart_rx_it();
    }

    arm_tick();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    uart_tx_ready = 1;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (!rx_packet_ready)
    {
      if (rx_index < RX_BUFFER_MAX)
      {
        rx_buffer[rx_index++] = rx_byte;
      }
      else
      {
        rx_index = 0U;
        rx_frame_len = 0U;
        (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1U);
        return;
      }

      /* Extract frame length from bytes 3-4 (little-endian) */
      if (rx_index == 5U && rx_frame_len == 0U)
      {
        rx_frame_len = (uint16_t)rx_buffer[3] | ((uint16_t)rx_buffer[4] << 8);

        if (rx_frame_len > (RX_BUFFER_MAX - 5U))
        {
          rx_packet_ready = 1U;
          return;
        }
      }

      /* Check if frame is complete */
      if (rx_index >= (5U + rx_frame_len) && rx_frame_len > 0U)
      {
        rx_packet_ready = 1;
      }
      else
      {
        (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1U);
      }
    }
  }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  uint8_t idx;

  for (idx = 0U; idx < STEPPER_COUNT; idx++)
  {
    stepper_motor_t *motor = &motors[idx];

    if (motor->htim->Instance != htim->Instance)
    {
      continue;
    }

    /* Each pulse is one step; keep a running position for the arm kinematics. */
    motor->position_steps += (motor->direction != 0U) ? 1 : -1;

    if (motor->move_active == 0U)
    {
      return;
    }

    if (motor->move_done_steps < motor->move_target_steps)
    {
      motor->move_done_steps++;
    }

    if (motor->move_ramp != 0U)
    {
      stepper_update_ramp(motor);
    }

    if (motor->move_done_steps >= motor->move_target_steps)
    {
      motor->move_active = 0U;
      motor->rate_hz = 0U;
      stepper_stop_pwm(motor);
    }
    return;
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
