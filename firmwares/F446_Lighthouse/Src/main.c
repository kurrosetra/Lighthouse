/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2019 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "stm_hal_serial.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* TODO Global Variables*/
#define DEBUG				1
#if DEBUG

#define UART_BUFSIZE	RING_BUFFER_SIZE
Ring_Buffer_t tx2Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx2Buffer = { { 0 }, 0, 0 };
TSerial debug = { &rx2Buffer, &tx2Buffer, &huart2 };
char buf[UART_BUFSIZE];
uint16_t bufLen;
char vt100_home[10];
#define DEBUG_LINE_MAX		25
char vt100_lineX[DEBUG_LINE_MAX][16];

#endif	//if DEBUG

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

const uint16_t pulse_data_max = 165;
const uint16_t pulse_midpoints[8] = { 187, 219, 250, 281, 312, 345, 375, 405 };
const uint16_t pulse_margin = 12;
//const uint16_t pulse_data_max = 55;
//const uint16_t pulse_midpoints[8] = { 62, 73, 83, 94, 104, 115, 125, 135 };
//const uint16_t pulse_margin = 4;

#define PULSE_BUFSIZE			100

typedef enum
{
	PULSE_RISE = 0,
	PULSE_FALL = 1
} PulseType;

typedef struct
{
	volatile uint16_t timestamp[PULSE_BUFSIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
} Pulse_Buffer_t;

#define ST_SWEEP_START_BIT			0
#define ST_SWEEP_SKIP_SYNC_BIT		1
#define ST_SWEEP_NSKIP_SYNC_BIT		2
#define ST_SWEEP_GOT_DATA_BIT		3

typedef struct
{
	uint8_t instance;
	uint8_t id_tem;
	Pulse_Buffer_t pulse;
	float angles[4];
	uint8_t state;
	uint16_t startNskipPulse;
	uint16_t startDataPulse;
} Sensor_t;
Sensor_t sensorL[4];

#define OOTX_BUFSIZE		256

typedef struct
{
	uint8_t waiting_for_preamble;
	uint8_t waiting_for_length;
	uint32_t accumulator;
	uint16_t accumulator_bits;
	uint16_t rx_bytes;
	uint16_t padding;
} Lighthouse_OOTX_Private_t;

typedef struct
{
	uint8_t completed;
	uint16_t length;
	uint8_t bytes[OOTX_BUFSIZE];
	Lighthouse_OOTX_Private_t private;
} Lighthouse_OOTX_t;
Lighthouse_OOTX_t ootx[4];

uint32_t ootx_reset_counter = 0;
uint32_t ootx_error_counter = 0;
uint32_t ootx_preamble_counter = 0;
uint32_t ootx_add_word_counter = 0;
uint32_t pulse_invalid_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* TODO Function Prototypes*/
static uint16_t findPulseLength(uint16_t start, uint16_t end);
//static float findAngle(uint16_t nskip, uint16_t start, uint16_t end);
static bool findAngle(uint16_t nskip, uint16_t start, uint16_t end, float *angle);
static void sensor_init();
static void sensorProcessing(Sensor_t *sensor);
static void sensorHandler();

static void pulse_init(Pulse_Buffer_t *buffer);
static void pulse_write_rise(Pulse_Buffer_t *buffer, uint16_t time);
static void pulse_write_fall(Pulse_Buffer_t *buffer, uint16_t time);
static bool pulse_buffer_available(Pulse_Buffer_t *buffer);
static bool pulse_read(Pulse_Buffer_t *buffer, uint16_t *rise, uint16_t *fall);

static void ootx_init(Lighthouse_OOTX_t *ootx);
static void ootx_reset(Lighthouse_OOTX_Private_t *ootxPrivate);
static void ootx_add_bit(Lighthouse_OOTX_t *ootx, uint8_t bit);
static void ootx_add_word(Lighthouse_OOTX_t *ootx, uint16_t word);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	sprintf(vt100_home, "\x1b[2J\x1b[H");
	unsigned char debugLine = 0;
	for ( debugLine = 0; debugLine < DEBUG_LINE_MAX; debugLine++ )
		sprintf(vt100_lineX[debugLine], "\x1b[%d;0H\x1b[2K", debugLine);

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_IWDG_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Initialization*/
	HAL_IWDG_Init(&hiwdg);
	HAL_IWDG_Refresh(&hiwdg);

	bufLen = sprintf(buf, "%sF446 Lighthouse Firmware!\r\n=========================\r\n",
			vt100_home);
	serial_write_str(&debug, buf, bufLen);

	serial_init(&debug);
	HAL_Delay(500);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	serial_init(&debug);

	sensor_init();
	ootx_reset_counter = 0;
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/* TODO Loop Variables*/

	uint32_t millis = 0;
	uint32_t timer = 0;
	uint32_t displayTimer = 0;

	uint16_t ootxCounter = 0;

	while (1) {
		float s[4][4];
		uint8_t i, j;

		millis = HAL_GetTick();
		HAL_IWDG_Refresh(&hiwdg);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO Begin LOOP*/
		if (millis >= timer) {
			timer = millis + 1000;

			if (HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin) == GPIO_PIN_RESET)
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}

		sensorHandler();
		if (millis >= displayTimer) {
			displayTimer = millis + 100;

			for ( i = 0; i < 4; i++ ) {
				for ( j = 0; j < 4; j++ )
					s[i][j] = sensorL[i].angles[j];
			}

			bufLen = sprintf(buf, "%slivetime= %d", vt100_lineX[3], (int) (millis / 1000));
			serial_write_str(&debug, buf, bufLen);

			bufLen = sprintf(buf, "%ssensor0:%s%.3f\t%.3f\t%.3f\t%.3f", vt100_lineX[5],
					vt100_lineX[6], s[0][0], s[0][1], s[0][2], s[0][3]);
			serial_write_str(&debug, buf, bufLen);
			bufLen = sprintf(buf, "%ssensor1:%s%.3f\t%.3f\t%.3f\t%.3f", vt100_lineX[7],
					vt100_lineX[8], s[1][0], s[1][1], s[1][2], s[1][3]);
			serial_write_str(&debug, buf, bufLen);
//			bufLen = sprintf(buf, "%ssensor2:%s%.3f\t%.3f\t%.3f\t%.3f", vt100_lineX[9],
//					vt100_lineX[10], s[2][0], s[2][1], s[2][2], s[2][3]);
//			serial_write_str(&debug, buf, bufLen);
//			bufLen = sprintf(buf, "%ssensor3:%s%.3f\t%.3f\t%.3f\t%.3f", vt100_lineX[11],
//					vt100_lineX[12], s[3][0], s[3][1], s[3][2], s[3][3]);
//			serial_write_str(&debug, buf, bufLen);

			bufLen = sprintf(buf, "%sDebug:%spulseInvalid= %d\tpreamble= %d | %d | %d",
					vt100_lineX[15], vt100_lineX[16], (int) pulse_invalid_counter,
					(int) ootx_preamble_counter, (int) ootx_reset_counter,
					(int) ootx_add_word_counter);
			serial_write_str(&debug, buf, bufLen);
			bufLen = sprintf(buf, "%s%d\t%X", vt100_lineX[17],
					(int) ootx[1].private.accumulator_bits, (int) ootx[1].private.accumulator);
			serial_write_str(&debug, buf, bufLen);

			bufLen = sprintf(buf, "%s", vt100_lineX[22]);
			serial_write_str(&debug, buf, bufLen);

			for ( int i = 0; i < 38; i++ ) {
				if (i % 16 == 15)
					bufLen = sprintf(buf, "%02X-%02X\r\n", i, ootx[1].bytes[i]);
				else
					bufLen = sprintf(buf, "%02X-%02X ", i, ootx[1].bytes[i]);
				serial_write_str(&debug, buf, bufLen);
			}

		}

		if (ootx[1].completed > 0) {
			bufLen = sprintf(buf, "%sootx counter= %d", vt100_lineX[18], ootxCounter++);
			serial_write_str(&debug, buf, bufLen);

			ootx[1].completed = 0;
		}
		/* TODO END LOOP*/
	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Reload = 1000;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 29;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xFFFF;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 29;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xffff;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 230400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC0 PC1 PC2 PC3
	 PC4 PC5 PC6 PC7
	 PC8 PC9 PC10 PC11
	 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4
			| GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
			| GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA7 PA8 PA9
	 PA10 PA11 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
			| GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB10
	 PB12 PB13 PB14 PB15
	 PB4 PB5 PB6 PB7
	 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_12
			| GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
			| GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO Function Declarations*/
void USART2_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}

static uint16_t findPulseLength(uint16_t start, uint16_t end)
{
	uint16_t ret = 0;
	if (end >= start)
		ret = end - start;
	else
		ret = 0xFFFF - start + end;

	return ret;
}

static bool findAngle(uint16_t nskip, uint16_t start, uint16_t end, float *angle)
{
	uint16_t _pulseStart = 0, _pulseEnd = 0;
	float _pulseMid = 0.0f;
	float _delta = 0.0f;

	_pulseStart = findPulseLength(nskip, start);
	_pulseEnd = findPulseLength(nskip, end);

	if (_pulseStart >= 25000 || _pulseEnd >= 25000)
		return false;
	_pulseMid = (float) (_pulseStart + _pulseEnd) / 2;

	/* 4000us -> 12000 ticks
	 * 8333us -> 24999 ticks
	 *
	 */
	_delta = _pulseMid - 12000.0;
	if (fabs(_delta) > 12000.0)
		return false;
//	_delta = _pulseMid - 4000.0;
//	if (fabs(_delta) > 4000.0)
//		return false;

//	*angle = _delta * M_PI / 24999.9;
	*angle = _delta * 180.0 / 24999.9;
//	*angle = _delta * 180.0 / 8333;

	return true;
}

static void ootx_init(Lighthouse_OOTX_t *ootx)
{
	ootx_reset(&ootx->private);
	ootx->completed = 0;
	ootx->length = 0;
	memset(ootx->bytes, 0, OOTX_BUFSIZE);
}

static void ootx_reset(Lighthouse_OOTX_Private_t *ootxPrivate)
{
	ootxPrivate->waiting_for_preamble = 1;
	ootxPrivate->waiting_for_length = 1;
	ootxPrivate->accumulator = 0;
	ootxPrivate->accumulator_bits = 0;
	ootxPrivate->rx_bytes = 0;
	ootx_reset_counter++;
	ootx_add_word_counter = 0;
}


static void ootx_add_bit(Lighthouse_OOTX_t *ootx, uint8_t bit)
{
	Lighthouse_OOTX_Private_t *oPrivate = &ootx->private;

	bit &= 0b1;

	/* add this bit to our incoming word */
	oPrivate->accumulator = (oPrivate->accumulator << 1) | bit;
	oPrivate->accumulator_bits++;

	if (oPrivate->waiting_for_preamble) {
		/* 17 zeros, followed by a 1 == 18 bits */
		if (oPrivate->accumulator_bits != 18)
			return;

		/* received preamble, start on data */
		if (oPrivate->accumulator == 0b1) {
			ootx_preamble_counter++;
			/* first we'll need the length */
			oPrivate->waiting_for_preamble = 0;
			oPrivate->waiting_for_length = 1;
			oPrivate->accumulator = 0;
			oPrivate->accumulator_bits = 0;
			return;
		}
		/* we've received 18 bits worth of preamble
		 * but it isnt a valid thing. hold onto the
		 * last 17 bits worth of data
		 */
		oPrivate->accumulator_bits--;
		oPrivate->accumulator &= 0x1FFFF;
		return;
	}

	/* we're receiving data!  accumulate until we get a sync bit */
	if (oPrivate->accumulator_bits < 17)
		return;

	if ((oPrivate->accumulator & 0b1) == 0) {
		/* no sync bit. go back into waiting for preamble mode */
//		if (oPrivate->rx_bytes >= 38) {
//			uint16_t fw_version = ((uint16_t) ootx->bytes[1] << 8) | ootx->bytes[0];
//
//			bufLen = sprintf(buf, "%sfirmware= 0x%X %d | %d\t%d", vt100_lineX[21], fw_version,
//					fw_version >> 5, fw_version & 0x1f, ootx->private.rx_bytes);
////			bufLen = sprintf(buf, "%s%X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X",
////					vt100_lineX[21], ootx->bytes[0], ootx->bytes[1], ootx->bytes[2], ootx->bytes[3],
////					ootx->bytes[4], ootx->bytes[5], ootx->bytes[6], ootx->bytes[7], ootx->bytes[8],
////					ootx->bytes[9], ootx->bytes[10], ootx->bytes[11], ootx->bytes[12], ootx->bytes[13],
////					ootx->bytes[14], ootx->bytes[15], ootx->bytes[16], ootx->bytes[17], ootx->bytes[18],
////					ootx->bytes[19]);
//			serial_write_str(&debug, buf, bufLen);
//		}
		ootx_reset(&ootx->private);
		return;
	}

	/* hurrah!  the sync bit was set */
	uint16_t _word = (oPrivate->accumulator >> 1);
	oPrivate->accumulator = 0;
	oPrivate->accumulator_bits = 0;

	ootx_add_word(ootx, _word);
	ootx_add_word_counter++;
}

static void ootx_add_word(Lighthouse_OOTX_t *ootx, uint16_t word)
{
	Lighthouse_OOTX_Private_t *oPrivate = &ootx->private;

	if (oPrivate->waiting_for_length) {
		ootx->length = (((word >> 8) & 0xFF) | ((word & 0xFF) << 8)) + 4;
		oPrivate->padding = ootx->length & 1;
		oPrivate->waiting_for_length = 0;
		oPrivate->rx_bytes = 0;

		/* error */
		if (ootx->length > OOTX_BUFSIZE)
			ootx_reset(&ootx->private);
//		if (ootx->length > OOTX_BUFSIZE) {
//			bufLen = sprintf(buf, "%sLenR= %d -> %d", vt100_lineX[20], word, (int) ootx->length);
//			serial_write_str(&debug, buf, bufLen);
//
//			ootx_reset(&ootx->private);
//		}
//		else {
//			bufLen = sprintf(buf, "%sLenS= %d -> %d", vt100_lineX[20], word, (int) ootx->length);
//			serial_write_str(&debug, buf, bufLen);
//		}

		return;
	}

	/* little endian; [Byte0][Byte1] */
	ootx->bytes[oPrivate->rx_bytes++] = (word >> 8) & 0xFF;
	ootx->bytes[oPrivate->rx_bytes++] = word & 0xFF;

	if (oPrivate->rx_bytes < ootx->length + oPrivate->padding)
		return;

	/* TODO check CRC32*/

	ootx->completed++;

	/* reset to wait for preamble */
	ootx_reset(&ootx->private);
}

static void pulse_init(Pulse_Buffer_t *buffer)
{
	buffer->head = buffer->tail = 0;
}

static void pulse_write_rise(Pulse_Buffer_t *buffer, uint16_t time)
{
	if (!bitRead(buffer->head, 0)) {
		buffer->timestamp[buffer->head] = time;
		buffer->head = (buffer->head + 1) % PULSE_BUFSIZE;
	}
	/* else, ignore it and wait until we got pulse fall timestamp */
}

static void pulse_write_fall(Pulse_Buffer_t *buffer, uint16_t time)
{
	if (bitRead(buffer->head, 0)) {
		buffer->timestamp[buffer->head] = time;
		buffer->head = (buffer->head + 1) % PULSE_BUFSIZE;
	}
}

static bool pulse_buffer_available(Pulse_Buffer_t *buffer)
{
	uint16_t delta = 0;

	if (buffer->head >= buffer->tail)
		delta = buffer->head - buffer->tail;
	else
		delta = buffer->head + PULSE_BUFSIZE - buffer->tail;

	if (delta > 1)
		return true;

	return false;
}

static bool pulse_read(Pulse_Buffer_t *buffer, uint16_t *rise, uint16_t *fall)
{
	if (pulse_buffer_available(buffer)) {
		if (!bitRead(buffer->tail, 0)) {
			*rise = buffer->timestamp[buffer->tail];
			buffer->tail = (buffer->tail + 1) % PULSE_BUFSIZE;
			*fall = buffer->timestamp[buffer->tail];
			buffer->tail = (buffer->tail + 1) % PULSE_BUFSIZE;

			return true;
		}
	}

	return false;
}

static void sensor_init()
{
	for ( uint8_t i = 0; i < 4; i++ ) {
		sensorL[i].instance = i;
		pulse_init(&sensorL[i].pulse);
		sensorL[i].state = 0;
		ootx_init(&ootx[i]);
		for ( uint8_t j = 0; j < 4; j++ )
			sensorL[i].angles[j] = 0.0f;
	}
}

static void sensorProcessing(Sensor_t *sensor)
{
	/* if there is a new pulse */
	uint16_t _pulseLen = 0;
	uint16_t timeRise = 0, timeFall = 0;
	uint8_t i = 0;
	uint8_t skip = 0, data = 0, axis = 0;
	bool _pulseValid = false;
	float _newAngles = 0.0f;

	if (pulse_read(&sensor->pulse, &timeRise, &timeFall)) {
		_pulseLen = findPulseLength(timeRise, timeFall);

		/* sync pulse detected, hopefully */
		if (_pulseLen > pulse_data_max) {
			/* new sweep and data has been found */
			if (bitRead(sensor->state, ST_SWEEP_GOT_DATA_BIT))
				sensor->state = 0;

			/* find type of sync pulse */
			for ( i = 0; i < 8; i++ ) {
				if ((_pulseLen >= pulse_midpoints[i] - pulse_margin)
						&& (_pulseLen <= pulse_midpoints[i] + pulse_margin)) {
					skip = (i >> 2) & 1;
					data = (i >> 1) & 1;
					axis = i & 1;
					_pulseValid = true;
					break;
				}
			}

			/* sync pulse detected */
			if (_pulseValid) {
				/* sync pulse #1 */
				if (bitRead(sensor->state,ST_SWEEP_START_BIT) == 0) {
					if (skip)
						bitSet(sensor->state, ST_SWEEP_SKIP_SYNC_BIT);
					else {
						bitSet(sensor->state, ST_SWEEP_NSKIP_SYNC_BIT);
						sensor->startNskipPulse = timeRise;
						sensor->id_tem = axis;
					}
					bitSet(sensor->state, ST_SWEEP_START_BIT);
				}
				/* sync pulse #2 */
				else {
					/* sync pulse #1 is skip */
					if ((skip == 0) && bitRead(sensor->state, ST_SWEEP_SKIP_SYNC_BIT)) {
						bitSet(sensor->state, ST_SWEEP_NSKIP_SYNC_BIT);
						sensor->startNskipPulse = timeRise;
						sensor->id_tem = axis | 0b10;
						ootx_add_bit(&ootx[sensor->instance], data);
					}
					/* sync pulse #1 is nskip */
					else if ((skip == 1) && bitRead(sensor->state, ST_SWEEP_NSKIP_SYNC_BIT)) {
						bitSet(sensor->state, ST_SWEEP_SKIP_SYNC_BIT);
						ootx_add_bit(&ootx[sensor->instance], data);
					}
					/* sync pulse #2 is invalid */
					else {
						sensor->state = 0;
						ootx_reset(&ootx[sensor->instance].private);
					}
				}
			}	//if (_pulseValid)
			else {
				sensor->state = 0;
				pulse_invalid_counter++;
			}
		}
		/* data pulse detected */
		else {
			/* double sync pulse is already detected */
			if (sensor->state & 0b111) {
				/* if data pulse has not been found yet */
				if (!bitRead(sensor->state, ST_SWEEP_GOT_DATA_BIT)) {
					/* calculate angle */
					if (findAngle(sensor->startNskipPulse, timeRise, timeFall, &_newAngles)) {
//						sensor->angles[sensor->id_tem] = _newAngles;
						sensor->angles[sensor->id_tem] = (sensor->angles[sensor->id_tem] * 0.75)
								+ (_newAngles * 0.25);
						bitSet(sensor->state, ST_SWEEP_GOT_DATA_BIT);
						sensor->startDataPulse = timeRise;
					}

				}
//					/* if data pulse has been found */
//					else
//						sensor->angles[sensor->id_tem] = findAngle(sensor->startNskipPulse,
//								sensor->startDataPulse, timeFall);
			}
		}
	}	//if (pulse_read(&sensor->pulse, &timeRise, &timeFall))

}

static void sensorHandler()
{
	for ( uint8_t i = 0; i < 4; i++ )
		sensorProcessing(&sensorL[i]);
}

void TIM2_IRQHandler(void)
{
	TIM_HandleTypeDef *htim = &htim2;
	uint32_t isrflags = READ_REG(htim->Instance->SR);
	uint16_t _timestamp = htim->Instance->CNT;
	Pulse_Buffer_t *pulseBuffer1 = &sensorL[1].pulse;

	/* Capture compare 1 event */
	if ((isrflags & TIM_FLAG_CC1) != RESET) {
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
		/* Input capture event */
		if (HAL_GPIO_ReadPin(SENSOR_0_GPIO_Port, SENSOR_0_Pin) == GPIO_PIN_RESET)
			pulse_write_rise(pulseBuffer1, _timestamp);
		else
			pulse_write_fall(pulseBuffer1, _timestamp);
	}

}

void TIM3_IRQHandler(void)
{
	TIM_HandleTypeDef *htim = &htim3;
	uint32_t isrflags = READ_REG(htim->Instance->SR);
	uint16_t _timestamp = htim->Instance->CNT;
	Pulse_Buffer_t *pulseBuffer0 = &sensorL[0].pulse;

	/* Capture compare 1 event */
	if ((isrflags & TIM_FLAG_CC1) != RESET) {
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
		/* Input capture event */
		if (HAL_GPIO_ReadPin(SENSOR_1_GPIO_Port, SENSOR_1_Pin) == GPIO_PIN_RESET)
			pulse_write_rise(pulseBuffer0, _timestamp);
		else
			pulse_write_fall(pulseBuffer0, _timestamp);
	}

}

/* TODO END of Function Declarations*/
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
