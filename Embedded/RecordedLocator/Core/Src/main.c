/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include  <errno.h>
#include <string.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES 1024
#define SIN_FREQUENCY 200
#define DIFF_THRESHOLD 30
#define AVG_WINDOW 5

#define F_MIN 20
#define F_MAX 2000

#define START_RECORD 0
#define STOP_RECORD 1
#define CLEAR_RECORD 2
#define SET_FREQUENCY_RANGE 3
#define SET_LOCATION_MODE 4
#define START_LOCATING 5
#define STOP_LOCATING 6
#define SET_COORDNATES 7
#define DEBUG 8

#define MODE_RECORDING 0
#define MODE_FREQUENCY 1

#define SENSOR_DISTANCE 0.04
#define SPEED 343.0

#define ADC_SLOW 0
#define ADC_FAST 1

union int_float {
	uint16_t int_val;
	float32_t float_val;
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
static void to_float(uint16_t*, float32_t*, uint32_t);

//Overwrite recording data
void do_recording();

//Convert data to float
void convert_recording();

//Reverse recording
void reverse_recording();

//Correlate recording with recording
void autocorrelate_recording();

//1 if recording is present in given sample data
uint8_t recording_is_present(uint16_t* ADC_a);

//Perform FFT on ADC_a and ADC_b and return an angle if the dominant frequency of ADC_a is within low_thresh and high_thresh, NAN otherwise
float32_t do_fft(uint16_t* ADC_a, uint16_t* ADC_b, int low_thresh, int high_thresh);

//Convert time difference of arrival to angle value between -pi/2 to pi/2
float32_t tdoa_to_angle(float32_t tdoa, float32_t sensor_distance, float32_t speed);

//1 if the dominant frequency in ADC_a is within low_thresh and high_thresh
uint8_t frequency_is_present(uint16_t* ADC_a, uint32_t low_thresh, uint32_t high_thresh);

//Find the tdoa of the signal of ADC_a and ADC_b (cross correlation)
float32_t find_tdoa(uint16_t* ADC_a, uint16_t* ADC_b);

//Send angles to phone
void send_angles(int8_t angle_tb, int8_t angle_lr);

//Perfrom rolling average on given angle and past angle values
int8_t get_angle(int8_t, int8_t*, uint8_t*);

void set_adc_fast();
void set_adc_slow();

void do_samples(uint8_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ADC_top[SAMPLES];
uint16_t ADC_bottom[SAMPLES];
uint16_t ADC_right[SAMPLES];
uint16_t ADC_left[SAMPLES];
uint16_t ADC_all[SAMPLES * 4];

float32_t a_float[SAMPLES * 2];
float32_t b_float[SAMPLES * 2];
float32_t a_mag[SAMPLES / 2];

float32_t a_phase[SAMPLES];
float32_t b_phase[SAMPLES];

int8_t tb_avg_buffer[AVG_WINDOW];
int8_t lr_avg_buffer[AVG_WINDOW];
int8_t tb_avg_i = 0;
int8_t lr_avg_i = 0;


union int_float recording[SAMPLES / 2];
float32_t recording_autocorrelation[SAMPLES];
float32_t recording_max_correlation;
arm_cfft_instance_f32 fft_instance;

uint8_t startLocating = 0;
uint8_t start_recording = 0;
uint8_t location_mode = MODE_FREQUENCY;

uint16_t fL = F_MIN;
uint16_t fH = F_MAX;

uint8_t command[5];

uint8_t samples_done = 0;

float32_t sampling_frequency = 153284.0;

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
  arm_cfft_init_f32(&fft_instance, SAMPLES);


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ADC_all, SAMPLES * 4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

	  HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, command, 5);

	  if(location_mode == MODE_RECORDING) {
		  if(start_recording) {
			  do_recording();
			  convert_recording();
			  autocorrelate_recording();
			  start_recording = 0;
		  }
		  if(startLocating) {
			  //TODO Change this
			  if(recording_is_present(ADC_top)) {
				  float32_t diff_tb = do_fft(ADC_top, ADC_bottom, 0, INT_MAX);
				  float32_t diff_lr = do_fft(ADC_right, ADC_left, 0, INT_MAX);

				  uint16_t angle_tb = (uint16_t) tdoa_to_angle(diff_tb, SENSOR_DISTANCE, SPEED);
				  uint16_t angle_lr = (uint16_t) tdoa_to_angle(diff_lr, SENSOR_DISTANCE, SPEED);

				  send_angles(angle_tb, angle_lr);

			  }


		  }
	  }
	  if(location_mode == MODE_FREQUENCY) {
		  if(startLocating) {
			  set_adc_slow();
			  while(samples_done == 0) {};

			  if(frequency_is_present(ADC_right, fL, fH)) {
				  set_adc_fast();
				  while(samples_done == 0) {};
				  float32_t diff_tb = find_tdoa(ADC_top, ADC_bottom);
				  float32_t diff_lr = find_tdoa(ADC_left, ADC_right);


				  int8_t angle_tb = (int8_t) (tdoa_to_angle(diff_tb, SENSOR_DISTANCE, SPEED) * (180.0/M_PI));
				  int8_t angle_lr = (int8_t) (tdoa_to_angle(diff_lr, SENSOR_DISTANCE, SPEED) * (180.0/M_PI));

				  send_angles(angle_tb, angle_lr);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int _write(int file, char *data, int len)
{
   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      errno = EBADF;
      return -1;
   }

   // arbitrary timeout 1000
   HAL_StatusTypeDef status =
      HAL_UART_Transmit(&huart2, (uint8_t*)data, len, 1000);

   // return # of bytes written - as best we can tell
   return (status == HAL_OK ? len : 0);
}

void set_adc_fast() {
  HAL_ADC_Stop(&hadc2);
  HAL_ADCEx_MultiModeStop_DMA(&hadc1);

	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
		Error_Handler();
	}
	samples_done = 0;
	sampling_frequency = 153284.0;

	HAL_Delay(2);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ADC_all, SAMPLES * 4);
}

void set_adc_slow() {
	  HAL_ADC_Stop(&hadc2);
	  HAL_ADCEx_MultiModeStop_DMA(&hadc1);

	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
		Error_Handler();
	}
	samples_done = 0;
	sampling_frequency = 39220.0;

	HAL_Delay(2);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ADC_all, SAMPLES * 4);


}

void do_samples(uint8_t mode) {
	if(mode == ADC_SLOW) {
		set_adc_slow();
	}
	else if(mode == ADC_FAST) {
		set_adc_fast();
	}

	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc2, ADC_all, SAMPLES * 4);
	samples_done = 0;
	while(!samples_done) {}
}

float32_t tdoa_to_angle(float32_t tdoa, float32_t sensor_distance, float32_t speed) {
	float32_t a = (speed * tdoa) / sensor_distance;
	float32_t angle = asin(a);
	return angle;
}

int8_t get_angle(int8_t angle, int8_t* angle_avg, uint8_t* avg_i) {
	angle_avg[*avg_i] = angle;
	*avg_i = (*avg_i) + 1;
	if(*avg_i > AVG_WINDOW) {
		*avg_i = 0;
	}

	int16_t sum = 0;

	for(int i = 0; i < AVG_WINDOW; i++) {
		sum += angle_avg[i];
	}

	return sum / AVG_WINDOW;
}

void do_recording() {
	for(int i = 0; i < SAMPLES / 2; i++) {
//		recording[i].int_val =  (uint16_t) ((sin(2 * M_PI * SIN_RECORDING_FREQUENCY * ((double)i / SIN_SAMPLING_RATE)) + 1) * (double) ((1023 + 1) / 2));	}
	}
}

void convert_recording() {

	float32_t max = 0.0;

	for(int i = 0; i < SAMPLES / 2; i++) {
		recording[i].float_val = (float32_t) ((float32_t)recording[i].int_val);
		if(fabs(recording[i].float_val) > max) {
			max = fabs(recording[i].float_val);
		}
	}

	for(int i = 0; i < SAMPLES / 2; i++) {
		recording[i].float_val = recording[i].float_val / (max / 2) - 1;
	}



}

void reverse_recording() {
	int i = 0; int j = SAMPLES / 2 - 1;
	for(; i < j; ++i, --j) {
		recording[i].int_val = recording[j].int_val;
	}

}

void autocorrelate_recording() {
	arm_correlate_f32((const float32_t*) recording, SAMPLES, (const float32_t*) recording, SAMPLES, recording_autocorrelation);
	static uint32_t max_index;
	arm_max_f32(recording_autocorrelation, SAMPLES, &recording_max_correlation, &max_index);
}


uint8_t recording_is_present(uint16_t* ADC_a) {
	static float32_t recording_samples_correlation[SAMPLES * 2];
	static float32_t a_float[SAMPLES];

	memset(recording_samples_correlation, 0, SAMPLES * 2 * sizeof(float32_t));
	memset(a_float, 0, SAMPLES * sizeof(float32_t));

	to_float(ADC_a, a_float, SAMPLES);

	arm_correlate_f32(a_float, SAMPLES, (float32_t*) recording, SAMPLES / 2, recording_samples_correlation);


	float32_t max_corr;
	arm_max_no_idx_f32(recording_samples_correlation, SAMPLES * 2, &max_corr);

	float32_t corr_diff = fabs(recording_max_correlation - max_corr);

	if(corr_diff < DIFF_THRESHOLD) {
		return 1;
	}


	return 0;

}

uint8_t frequency_is_present(uint16_t* ADC_a, uint32_t low_thresh, uint32_t high_thresh) {
	memset(a_float, 0, SAMPLES * 2 * sizeof(float32_t));
	memset(a_mag, 0, SAMPLES / 2  * sizeof(float32_t));
	to_float(ADC_a, a_float, SAMPLES);


	arm_cfft_init_f32(&fft_instance, SAMPLES);
	arm_cfft_f32(&fft_instance, a_float, 0, 1);
	arm_cmplx_mag_f32(a_float, a_mag, SAMPLES / 2);

	float32_t max;
	uint32_t index;
	arm_max_f32(a_mag, SAMPLES / 2, &max, &index);


	int frequency = (index * sampling_frequency) / SAMPLES / 2;
	return frequency >= low_thresh && frequency <= high_thresh;


}

float32_t find_tdoa(uint16_t* ADC_a, uint16_t* ADC_b) {
	static float32_t samples_correlation[SAMPLES * 2];
	memset(a_float, 0, SAMPLES * 2 * sizeof(float32_t));
	memset(b_float, 0, SAMPLES * 2 * sizeof(float32_t));
	memset(samples_correlation, 0, SAMPLES * sizeof(float32_t));


	to_float(ADC_a, a_float, SAMPLES);
	to_float(ADC_b, b_float, SAMPLES);

	arm_correlate_f32(a_float, SAMPLES, b_float, SAMPLES, samples_correlation);

	static uint32_t max_correlation_i;
	static float32_t max_correlation;

	arm_max_f32(samples_correlation, SAMPLES * 2, &max_correlation, &max_correlation_i);

	return (((float32_t)max_correlation_i) - (SAMPLES))/sampling_frequency;


}

void send_angles(int8_t angle_tb, int8_t angle_lr) {
	uint8_t data[] = {SET_COORDNATES};
    HAL_UART_Transmit(&huart2, (uint8_t*)data, 1, 1000);
	int8_t data2[] = {angle_tb, angle_lr};
	HAL_UART_Transmit(&huart2, data2, sizeof(int8_t) * 2, 1000);
	//printf("%d, %d\n\r", angle_tb, angle_lr);
}

float32_t do_fft(uint16_t* ADC_a, uint16_t* ADC_b, int low_thresh, int high_thresh) {

	memset(a_float, 0, SAMPLES * 2 * sizeof(float32_t));
	memset(b_float, 0, SAMPLES * 2 * sizeof(float32_t));
	memset(a_mag, 0, SAMPLES / 2  * sizeof(float32_t));
	memset(a_phase, 0, SAMPLES * sizeof(float32_t));
	memset(b_phase, 0, SAMPLES * sizeof(float32_t));
	to_float(ADC_a, a_float, SAMPLES);
	to_float(ADC_b, b_float, SAMPLES);

	arm_cfft_init_f32(&fft_instance, SAMPLES);
	arm_cfft_f32(&fft_instance, a_float, 0, 1);
	arm_cmplx_mag_f32(a_float, a_mag, SAMPLES / 2);
	a_mag[0] = 0;

	//calculate phase for each bin
	for(int i = 0; i < SAMPLES * 2; i+=2) {
		a_phase[i/2] = atan2(a_float[i + 1], a_float[i]);
	}

	arm_cfft_init_f32(&fft_instance, SAMPLES);
	arm_cfft_f32(&fft_instance, b_float, 0, 1);

	//calculate phase for each bin
	for(int i = 0; i < SAMPLES * 2; i+=2) {
		b_phase[i/2] = atan2(b_float[i + 1],  b_float[i]);
	}

	float32_t max;
	uint32_t index;
	arm_max_f32(a_mag, SAMPLES / 2, &max, &index);

	int frequency = (index * sampling_frequency) / SAMPLES / 2;
	if(frequency >= low_thresh && frequency <= high_thresh) {
		float32_t a_phase_max = a_phase[index];
		float32_t b_phase_max = b_phase[index];
		float32_t a = ((SPEED/frequency)/(2 * M_PI * SENSOR_DISTANCE)) * (a_phase_max - b_phase_max);
		float32_t tb_t_diff = acos(a);//((a_phase_max - b_phase_max) * SAMPLES)/(2 * M_PI);
		return tb_t_diff;
	}
	return NAN;

}

void to_float(uint16_t* int_array, float32_t* float_array, uint32_t length) {

	float32_t max = 0.0;
	float32_t min = 10000.0; //ADC values will never go above 1024 anyway
	for(int i = 0; i < length; i++) {
		float_array[i] = ((float32_t)int_array[i]);
		if(fabs(float_array[i]) > max) {
			max = fabs(float_array[i]);
		}
		if(fabs(float_array[i]) < min) {
			min = fabs(float_array[i]);
		}
	}
	for(int i = 0; i < length; i++) {
		float_array[i] = (float_array[i] - min) * (2) /(max - min) - 1;
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

  HAL_ADC_Stop(&hadc2);
  HAL_ADCEx_MultiModeStop_DMA(&hadc1);

	for(int i = 0; i < SAMPLES * 4; i += 4) {
		ADC_top[i/4] = ADC_all[i];
		ADC_left[i/4] = ADC_all[i+1];
		ADC_right[i/4] = ADC_all[i+2];
		ADC_bottom[i/4] = ADC_all[i+3];
	}
	samples_done = 1;
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ADC_all, SAMPLES * 4);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	  if(command[0] == SET_FREQUENCY_RANGE) {
		  fL = ((uint16_t)(command[1] << 8)) | command[2];
		  fH = ((uint16_t)(command[3] << 8)) | command[4];
	  }
	  if(command[0] == START_LOCATING) {
		  startLocating = 1;
	  }
	  if(command[0] == STOP_LOCATING) {
		  startLocating = 0;
	  }

	  if(command[0] == START_RECORD) {
		  start_recording = 1;
	  }
	  if(command[0] == STOP_RECORD) {
		  start_recording = 0;
	  }
	  if(command[0] == SET_LOCATION_MODE) {
		  location_mode = command[1];
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
