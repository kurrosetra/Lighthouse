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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "stm_hal_serial.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* TODO Global Variables*/
#define DEBUG				1
#define TEST_INPUT_CAPTURE	0

#if DEBUG

#define UART_BUFSIZE	RING_BUFFER_SIZE
Ring_Buffer_t tx3Buffer = { { 0 }, 0, 0 };
Ring_Buffer_t rx3Buffer = { { 0 }, 0, 0 };
TSerial debug = { &rx3Buffer, &tx3Buffer, &huart3 };
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

#if TEST_INPUT_CAPTURE==0

const uint16_t pulse_data_max = 165;
const uint16_t pulse_midpoints[8] = { 187, 219, 250, 281, 312, 345, 375, 405 };
const uint16_t pulse_margin = 12;
//const uint16_t pulse_data_max = 55;
//const uint16_t pulse_midpoints[8] = { 62, 73, 83, 94, 104, 115, 125, 135 };
//const uint16_t pulse_margin = 4;

#define PULSE_BUFSIZE			100

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
#endif	//if TEST_INPUT_CAPTURE==0

uint32_t ootx_reset_counter = 0;
uint32_t ootx_error_counter = 0;
uint32_t ootx_preamble_counter = 0;
uint32_t pulse_invalid_counter = 0;

#if TEST_INPUT_CAPTURE
volatile uint32_t inputCaptureVal[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile ITStatus inputCaptureFlag[8] = {RESET, RESET, RESET, RESET, RESET, RESET, RESET, RESET};
#endif	//if TEST_INPUT_CAPTURE

uint16_t errorCounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* TODO Function Prototypes*/
#if TEST_INPUT_CAPTURE==0
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

#endif	//if TEST_INPUT_CAPTURE==0

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

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
#if DEBUG
	sprintf(vt100_home, "\x1b[2J\x1b[H");
	unsigned char debugLine = 0;
	for ( debugLine = 0; debugLine < DEBUG_LINE_MAX; debugLine++ )
		sprintf(vt100_lineX[debugLine], "\x1b[%d;0H\x1b[2K", debugLine);
#endif	//if DEBUG

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_IWDG_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Initialization*/
	HAL_IWDG_Init(&hiwdg);
	HAL_IWDG_Refresh(&hiwdg);

#if DEBUG
	bufLen = sprintf(buf, "%sF103 Lighthouse Firmware!\r\n=========================\r\n",
			vt100_home);
//	bufLen = sprintf(buf, "F103 Lighthouse Firmware!\r\n=========================\r\n");
	serial_write_str(&debug, buf, bufLen);

	serial_init(&debug);
	HAL_Delay(500);
	HAL_IWDG_Refresh(&hiwdg);
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
//	bufLen = sprintf(buf, "done!\r\n=========================\r\n");
//	serial_write_str(&debug, buf, bufLen);

	serial_init(&debug);
#endif	//if DEBUG

#if TEST_INPUT_CAPTURE==0
	sensor_init();
#endif	//if TEST_INPUT_CAPTURE==0

	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t millis = 0;
	uint32_t timer = 0;
	uint32_t displayTimer = 0;

#if TEST_INPUT_CAPTURE==0 && DEBUG==1
	uint16_t ootxCounter = 0;
#endif	//if TEST_INPUT_CAPTURE==0

	while (1) {
		float s[4][4];
		uint8_t i, j;

		millis = HAL_GetTick();
		HAL_IWDG_Refresh(&hiwdg);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO BEGIN LOOP*/
		if (millis >= timer) {
			timer = millis + 1000;

			if (HAL_GPIO_ReadPin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin) == GPIO_PIN_RESET)
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
		}
#if DEBUG

#if TEST_INPUT_CAPTURE==0
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
			bufLen = sprintf(buf, "%ssensor2:%s%.3f\t%.3f\t%.3f\t%.3f", vt100_lineX[9],
					vt100_lineX[10], s[2][0], s[2][1], s[2][2], s[2][3]);
			serial_write_str(&debug, buf, bufLen);
			bufLen = sprintf(buf, "%ssensor3:%s%.3f\t%.3f\t%.3f\t%.3f", vt100_lineX[11],
					vt100_lineX[12], s[3][0], s[3][1], s[3][2], s[3][3]);
			serial_write_str(&debug, buf, bufLen);

			bufLen = sprintf(buf, "%sDebug:%spulseInvalid= %d\tpreamble= %d | %d", vt100_lineX[15],
					vt100_lineX[16], (int) pulse_invalid_counter, (int) ootx_preamble_counter,
					(int) ootx_reset_counter);
			serial_write_str(&debug, buf, bufLen);
			bufLen = sprintf(buf, "%s%d\t%X", vt100_lineX[17],
					(int) ootx[0].private.accumulator_bits, (int) ootx[0].private.accumulator);
			serial_write_str(&debug, buf, bufLen);

		}

		if (ootx[1].completed > 0) {
			bufLen = sprintf(buf, "%sootx counter= %d", vt100_lineX[20], ootxCounter++);
			serial_write_str(&debug, buf, bufLen);

			ootx[1].completed = 0;
		}
#endif	//if TEST_INPUT_CAPTURE==0

#if TEST_INPUT_CAPTURE

		if (inputCaptureFlag[0] == SET) {
			inputCaptureFlag[0] = RESET;

			bufLen = sprintf(buf, "%ss0= %d", vt100_lineX[3], (int) inputCaptureVal[0]);

			serial_write_str(&debug, buf, bufLen);
		}
		else if (inputCaptureFlag[1] == SET) {
			inputCaptureFlag[1] = RESET;

			bufLen = sprintf(buf, "%ss1= %d", vt100_lineX[4], (int) inputCaptureVal[1]);

			serial_write_str(&debug, buf, bufLen);
		}
		else if (inputCaptureFlag[2] == SET) {
			inputCaptureFlag[2] = RESET;

			bufLen = sprintf(buf, "%ss2= %d", vt100_lineX[5], (int) inputCaptureVal[2]);

			serial_write_str(&debug, buf, bufLen);
		}
		else if (inputCaptureFlag[3] == SET) {
			inputCaptureFlag[3] = RESET;

			bufLen = sprintf(buf, "%ss3= %d", vt100_lineX[6], (int) inputCaptureVal[3]);

			serial_write_str(&debug, buf, bufLen);
		}
		if (inputCaptureFlag[4] == SET) {
			inputCaptureFlag[4] = RESET;

			bufLen = sprintf(buf, "%ss4= %d", vt100_lineX[7], (int) inputCaptureVal[4]);

			serial_write_str(&debug, buf, bufLen);
		}
		else if (inputCaptureFlag[5] == SET) {
			inputCaptureFlag[5] = RESET;

			bufLen = sprintf(buf, "%ss5= %d", vt100_lineX[8], (int) inputCaptureVal[5]);

			serial_write_str(&debug, buf, bufLen);
		}
		else if (inputCaptureFlag[6] == SET) {
			inputCaptureFlag[6] = RESET;

			bufLen = sprintf(buf, "%ss6= %d", vt100_lineX[9], (int) inputCaptureVal[6]);

			serial_write_str(&debug, buf, bufLen);
		}
		else if (inputCaptureFlag[7] == SET) {
			inputCaptureFlag[7] = RESET;

			bufLen = sprintf(buf, "%ss7= %d", vt100_lineX[10], (int) inputCaptureVal[7]);

			serial_write_str(&debug, buf, bufLen);
		}

#endif	//if TEST_INPUT_CAPTURE
#endif	//if DEBUG

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

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
	hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
	hiwdg.Init.Reload = 625;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 23;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0xffff;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
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
	htim2.Init.Prescaler = 23;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xFFFF;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
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
	htim3.Init.Prescaler = 23;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xffff;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 23;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0xffff;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_IC_Init(&htim4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

	huart3.Instance = USART3;
	huart3.Init.BaudRate = 230400;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : LED_BUILTIN_Pin */
	GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC14 PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA2 PA3 PA4 PA5
	 PA10 PA11 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10
			| GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB12
	 PB13 PB14 PB15 PB3
	 PB4 PB5 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_12 | GPIO_PIN_13
			| GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8
			| GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO Function Declarations*/
#if DEBUG
void USART3_IRQHandler(void)
{
	USARTx_IRQHandler(&debug);
}
#endif	//if DEBUG

#if TEST_INPUT_CAPTURE==0

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
	if (oPrivate->accumulator_bits != 17)
		return;

	if ((oPrivate->accumulator & 0b1) == 0) {
		/* no sync bit. go back into waiting for preamble mode */
		ootx_reset(&ootx->private);
		return;
	}

	/* hurrah!  the sync bit was set */
	uint16_t _word = oPrivate->accumulator >> 1;
	oPrivate->accumulator = 0;
	oPrivate->accumulator_bits = 0;

	ootx_add_word(ootx, _word);
}

static void ootx_add_word(Lighthouse_OOTX_t *ootx, uint16_t word)
{
	Lighthouse_OOTX_Private_t *oPrivate = &ootx->private;

	if (oPrivate->waiting_for_length) {
		ootx->length = word + 4;	//add in the CRC32 length
		oPrivate->padding = ootx->length & 1;
		oPrivate->waiting_for_length = 0;
		oPrivate->rx_bytes = 0;

		/* error */
		if (ootx->length > OOTX_BUFSIZE)
			ootx_reset(&ootx->private);

		return;
	}

	ootx->bytes[oPrivate->rx_bytes++] = (word >> 8) & 0xFF;
	ootx->bytes[oPrivate->rx_bytes++] = word & 0xFF;

	if (oPrivate->rx_bytes < ootx->length + oPrivate->padding)
		return;

	/* TODO check CRC32*/

	ootx->completed = 1;

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
					else if ((skip == 1) && bitRead(sensor->state, ST_SWEEP_NSKIP_SYNC_BIT))
						bitSet(sensor->state, ST_SWEEP_SKIP_SYNC_BIT);
					/* sync pulse #2 is invalid */
					else
						sensor->state = 0;
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

//void TIM1_IRQHandler(void)
void TIM1_CC_IRQHandler(void)
{
	TIM_HandleTypeDef *htim = &htim1;
	uint16_t _timestamp = htim->Instance->CNT;
	Pulse_Buffer_t *pulseBuffer = &sensorL[0].pulse;

	/* Capture compare 1 event */
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1) != RESET) {
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1) != RESET) {
			{
				__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
				/* Input capture event */
				pulse_write_rise(pulseBuffer, _timestamp);
			}
		}
	}
	/* Capture compare 2 event */
	if (__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2) != RESET) {
		if (__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC2) != RESET) {
			__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
			/* Input capture event */
			pulse_write_fall(pulseBuffer, _timestamp);
		}
	}
}

void TIM2_IRQHandler(void)
{
	TIM_HandleTypeDef *htim = &htim2;
	uint32_t isrflags = READ_REG(htim->Instance->SR);
	uint16_t _timestamp = htim->Instance->CNT;
	Pulse_Buffer_t *pulseBuffer = &sensorL[1].pulse;

	/* Capture compare 1 event */
	if ((isrflags & TIM_FLAG_CC1) != RESET) {
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
		/* Input capture event */
		pulse_write_rise(pulseBuffer, _timestamp);
	}
	/* Capture compare 2 event */
	if ((isrflags & TIM_FLAG_CC2) != RESET) {
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
		/* Input capture event */
		pulse_write_fall(pulseBuffer, _timestamp);
	}
}

void TIM3_IRQHandler(void)
{
	TIM_HandleTypeDef *htim = &htim3;
	uint32_t isrflags = READ_REG(htim->Instance->SR);
	uint16_t _timestamp = htim->Instance->CNT;
	Pulse_Buffer_t *pulseBuffer = &sensorL[2].pulse;

	/* Capture compare 1 event */
	if ((isrflags & TIM_FLAG_CC1) != RESET) {
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
		/* Input capture event */
		pulse_write_rise(pulseBuffer, _timestamp);
	}
	/* Capture compare 2 event */
	if ((isrflags & TIM_FLAG_CC2) != RESET) {
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
		/* Input capture event */
		pulse_write_fall(pulseBuffer, _timestamp);
	}
}

void TIM4_IRQHandler(void)
{
	TIM_HandleTypeDef *htim = &htim4;
	uint32_t isrflags = READ_REG(htim->Instance->SR);
	uint16_t _timestamp = htim->Instance->CNT;
	Pulse_Buffer_t *pulseBuffer = &sensorL[3].pulse;

	/* Capture compare 1 event */
	if ((isrflags & TIM_FLAG_CC1) != RESET) {
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
		/* Input capture event */
		pulse_write_rise(pulseBuffer, _timestamp);
	}
	/* Capture compare 2 event */
	if ((isrflags & TIM_FLAG_CC2) != RESET) {
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
		/* Input capture event */
		pulse_write_fall(pulseBuffer, _timestamp);
	}
}

#endif	//if TEST_INPUT_CAPTURE==0

#if TEST_INPUT_CAPTURE
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t _timestamp = htim->Instance->CNT;
	htim->Instance->CNT = 0;

	if (htim->Instance == TIM1) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			inputCaptureVal[0] = _timestamp;
			inputCaptureFlag[0] = SET;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			inputCaptureVal[1] = _timestamp;
			inputCaptureFlag[1] = SET;
		}
	}

	if (htim->Instance == TIM2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			inputCaptureVal[2] = _timestamp;
			inputCaptureFlag[2] = SET;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			inputCaptureVal[3] = _timestamp;
			inputCaptureFlag[3] = SET;
		}
	}

	if (htim->Instance == TIM3) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			inputCaptureVal[4] = _timestamp;
			inputCaptureFlag[4] = SET;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			inputCaptureVal[5] = _timestamp;
			inputCaptureFlag[5] = SET;
		}
	}

	if (htim->Instance == TIM4) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
			inputCaptureVal[6] = _timestamp;
			inputCaptureFlag[6] = SET;
		}
		else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			inputCaptureVal[7] = _timestamp;
			inputCaptureFlag[7] = SET;
		}
	}
}
#endif	//if TEST_INPUT_CAPTURE

/* TODO End of Function Declarations*/
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
