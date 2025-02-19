/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
#include "toggle_pins.h"
#include "temperature.h"
#include "calculate_angles.h"
// Ensure the USART handle is included
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_CANDIDATES 10
typedef struct {
    float rsia;
    float rsib;
} RSIValues;
typedef struct {
    float freq_est;   // Przybliżona częstotliwość odbieranego sygnału
    uint16_t dac_setting;  // Nastawienie DAC
    float vco_temp;   // Temperatura VCO
    float amp_1khz;   // Amplituda składowej 1 kHz
    float mean_amp;   // Średnia amplituda składowych
    float ratio;
} Candidate;

Candidate candidate_list[MAX_CANDIDATES];  // Tablica do zapisu
int candidate_count = 0;
uint32_t AD_RES_BUFFER[5];
#define FFT_SIZE 512
#define SAMPLE_RATE  128000
uint16_t AD_RES = 0;
uint32_t tab[512];
float tab_ADC[FFT_SIZE];
//float tab_ADC_new[128];
float fft_input[2*FFT_SIZE];
float fft_output[FFT_SIZE ];
float fft_amplitude[FFT_SIZE/2];
float temper;
uint32_t dac_value;

#define V25      0.76f  // V25 w Voltach
#define AVG_SLOPE 0.0025f  // 2.5mV/°C = 0.0025 V/°C

#define R25 10000.0f
#define B_COEFFICIENT 3971.0f
#define T25 298.15f
#define VCC 3.3f
#define R_FIXED 10000.0f

#define delta_err 0.0
#define Uoh 2.9f
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int _write(int file, char *ptr, int len)
{
    if (HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY) != HAL_OK)
    {
        // Optionally handle error, but do *not* call printf here!
        return -1;  // Indicate error
    }
    return len;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM13_Init(void);
/* USER CODE BEGIN PFP */
float read_voltage(ADC_HandleTypeDef *hadc, uint32_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float read_voltage(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;

    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }


    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK) {
        uint32_t adcValue = HAL_ADC_GetValue(hadc);
        float voltage = (adcValue * 3.3) / 4096;
        return voltage;
    }
    HAL_ADC_Stop(hadc);

    return 0;
}

RSIValues read_rsi_values_scan(ADC_HandleTypeDef *hadc)
{
    RSIValues results = {0};

    HAL_ADC_Start(hadc);

    // Oczekiwanie na zakończenie konwersji skanowania
    if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK)
    {
        // Pierwszy wynik: RSIA (ADC_CHANNEL_10)
        uint32_t adcValue = HAL_ADC_GetValue(hadc);
        results.rsia = (adcValue * 3.3f) / 4095.0f;
    }

    if (HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY) == HAL_OK)
    {
        // Drugi wynik: RSIB (ADC_CHANNEL_11)
        uint32_t adcValue = HAL_ADC_GetValue(hadc);
        results.rsib = (adcValue * 3.3f) / 4095.0f;
    }

    HAL_ADC_Stop(hadc);

    return results;
}

void perform_fft(void) {
    // Initialize RFFT instance
    arm_rfft_fast_instance_f32 S;
    if (arm_rfft_fast_init_f32(&S, FFT_SIZE) != ARM_MATH_SUCCESS) {
        printf("FFT Initialization Error!\n\r");
        return;
    }

    for (int i = 3;i<FFT_SIZE -3;i++){
    	tab_ADC[i]=(tab_ADC[i-2]+tab_ADC[i-1]+tab_ADC[i]+tab_ADC[i+1]+tab_ADC[i+2])/5;
    }

    arm_rfft_fast_f32(&S, tab_ADC, fft_output, 0);


    arm_cmplx_mag_f32(fft_output, fft_amplitude, FFT_SIZE / 2);

    // Find the dominant frequency
    float max_amplitude = 0.0f;
    uint32_t max_index = 0;
    for (int i = 2; i < FFT_SIZE / 2; i++) {
        if (fft_amplitude[i] > max_amplitude) {
            max_amplitude = fft_amplitude[i];
            max_index = i;
        }
    }

    // Compute the dominant frequency
    float dominant_frequency = (float)max_index * SAMPLE_RATE / FFT_SIZE;

    // Print results
    //printf("Dominant Frequency: %.2f Hz, Amplitude: %.3f\n\r", dominant_frequency, max_amplitude);
}


void analyze_fft_results(void) {
    int idx_1khz = (1000 * FFT_SIZE) / SAMPLE_RATE;  // Indeks składowej 1 kHz

    // Oblicz średnią amplitudę składowych
    float mean_amp = 0.0f;
    for (int i = 1; i < FFT_SIZE / 2; i++) {  // Pomijamy DC (i=0)
        mean_amp += fft_amplitude[i];
    }
    mean_amp /= (FFT_SIZE / 2 - 1);  // Średnia amplitud składowych

    float amp_1khz = fft_amplitude[idx_1khz];  // Pobierz amplitudę 1 kHz
    float ratio = amp_1khz / mean_amp;  // Oblicz stosunek amplitudy 1 kHz do średniej

    // Sprawdź, czy mamy mniej niż 10 kandydatów
    if (candidate_count < MAX_CANDIDATES) {
        // Dodaj nowego kandydata
        candidate_list[candidate_count].freq_est = (float) idx_1khz * SAMPLE_RATE / FFT_SIZE;
        candidate_list[candidate_count].dac_setting = HAL_DAC_GetValue(&hdac, DAC_CHANNEL_1);
        candidate_list[candidate_count].vco_temp = read_voltage(&hadc3, ADC_CHANNEL_1);
        candidate_list[candidate_count].amp_1khz = amp_1khz;
        candidate_list[candidate_count].mean_amp = mean_amp;
        candidate_list[candidate_count].ratio = ratio;
        candidate_count++;
    } else {
        // Szukamy najsłabszego kandydata (najmniejszy stosunek R)
        int worst_index = 0;
        float min_ratio = candidate_list[0].ratio;
        for (int i = 1; i < MAX_CANDIDATES; i++) {
            if (candidate_list[i].ratio < min_ratio) {
                min_ratio = candidate_list[i].ratio;
                worst_index = i;
            }
        }

        // Jeśli nowy kandydat ma lepszy stosunek R, zastępujemy najgorszy
        if (ratio > min_ratio) {
            candidate_list[worst_index].freq_est = (float) idx_1khz * SAMPLE_RATE / FFT_SIZE;
            candidate_list[worst_index].dac_setting = HAL_DAC_GetValue(&hdac, DAC_CHANNEL_1);
            candidate_list[worst_index].vco_temp = read_voltage(&hadc3, ADC_CHANNEL_1);
            candidate_list[worst_index].amp_1khz = amp_1khz;
            candidate_list[worst_index].mean_amp = mean_amp;
            candidate_list[worst_index].ratio = ratio;
        }
    }

    // Debug: wypisz listę kandydatów
    //printf("Lista najlepszych kandydatów:\n\r");
    for (int i = 0; i < candidate_count; i++) {
        /*printf("[%d] f=%.2f Hz, DAC=%d, Temp=%.2f, Amp1kHz=%.2f, MeanAmp=%.2f, R=%.2f\n\r",
               i,
               candidate_list[i].freq_est,
               candidate_list[i].dac_setting,
               candidate_list[i].vco_temp,
               candidate_list[i].amp_1khz,
               candidate_list[i].mean_amp,
               candidate_list[i].ratio);*/
    }
}
void Read_CPU_Temperature_DMA(void) {
    uint32_t rawValue = AD_RES_BUFFER[3];
    printf("%lu\n\r",AD_RES_BUFFER[3]);
    float v_sense = ((float)rawValue * 3.3f) / 4095.0f;  // Przeliczenie ADC na napięcie
    printf("v_sense %f.......",v_sense);
    float temperature = ((v_sense - V25) / AVG_SLOPE) + 25.0f;

    printf("CPU Temp: %.2f°C\n\r", temperature);
    temper=temperature;
}

void Heater_On(void) {
    printf("Grzałka ON\n\r");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);  // Włącz grzałkę
}

void Heater_Off(void) {
    printf("Grzałka OFF\n\r");
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);  // Wyłącz grzałkę
}

float Get_Thermistor_Resistance(float v_out) {
    return R_FIXED * (v_out / (VCC - v_out));
}

float Convert_Resistance_To_Temperature(float r_therm) {
    float temperature_kelvin = B_COEFFICIENT /
        (log(r_therm / R25) + (B_COEFFICIENT / T25));
    return temperature_kelvin - 273.15f;  // Konwersja do °C
}

float Read_Thermistor_Temperature(void) {
    float v_out = AD_RES_BUFFER[1] ;
    float v_out1 = 0.9416f*((AD_RES_BUFFER[1] * 3.3f) / 4095.0f);
    float r_therm = Get_Thermistor_Resistance(v_out1);
    float temp_celsius = Convert_Resistance_To_Temperature(r_therm);  // Oblicz temperaturę

    printf("Vout: %.3fV, Rtherm: %.1fΩ, Temp: %.2f°C\n\r", v_out, r_therm, temp_celsius);

    return temp_celsius;
}



void Control_Heater(void) {
    float current_temp = Read_Thermistor_Temperature();
    float target_temp = temper+2.0f;
    printf("Current temp: %.2f°C, Target: %.2f°C\n\r", current_temp, target_temp);

    if (current_temp < target_temp) {
        //Heater_On();
    } else {
        //Heater_Off();
    }
    //dac_value++;
    //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int32_t test_array[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  int32_t result = 0.0;
  arm_mean_q31(test_array, 10, &result);

/*
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
*/
  //HAL_TIM_PWM_START(&htim13,TIM_CHANNEL_1);

  dac_value = 1;

  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);  // Włączenie DAC
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 1241);  // Ustawienie 1V
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc2, tab, 512);
  if (HAL_ADC_Start_DMA(&hadc1, AD_RES_BUFFER, 5) != HAL_OK) {
      printf("Błąd startu ADC z DMA!\n\r");
  }
  HAL_ADC_Start_DMA(&hadc1, AD_RES_BUFFER, 5);
  Read_CPU_Temperature_DMA();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  float cos_alpha[4];

	  for (int i =1;i<4;i++){
		  toggle_pins();
		  HAL_ADC_Start_DMA(&hadc1, AD_RES_BUFFER, 5);
		  Read_CPU_Temperature_DMA();
		  printf("ADC CH10: %.2fV, CH11: %.2fV, CH12: %.2fV, CH13: %fV\n\r",
		  	  	  	                           (AD_RES_BUFFER[0] * 3.3f) / 4095.0f,
		  	  	  	                           0.9416f*((AD_RES_BUFFER[1] * 3.3f) / 4095.0f),
		  	  	  	                           (AD_RES_BUFFER[2] * 3.3f) / 4095.0f,
		  	  	  	                           (AD_RES_BUFFER[4] * 3.3f) / 4095.0f);

		  float detector_out = (AD_RES_BUFFER[2] * 3.3f) / 4095.0f;
		  float input = 2.0f * (1.0f - (detector_out / Uoh)) + delta_err;
		  float value = asinf(input);
		  cos_alpha[i]= value;



		  for (int i = 0;i<512;i++){
		  	  		tab_ADC[i]=((tab[i] * 3.3f) / 4095.0f);
		  	  		//printf("%.2f  %.2f\n\r",tab_ADC[i], (tab[1] * 3.3f) / 4095.0f);
		  	  	}

		  perform_fft();
		  analyze_fft_results();

	  }
	  AngleResults angle = calculate_angles(cos_alpha);
	  //_HAL_TIM_SET_COMPARE(&htim13,TIM_CHANNEL_1,1000);

	  	for (int i = 0;i<512	;i++){
	  		//tab_ADC[i]=((tab[1] * 3.3f) / 4095.0f);
	  		//printf("%.2f  \n\r",tab_ADC[i]);
	  	}


	  	Control_Heater();
	  	HAL_Delay(1000);


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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 71;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

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
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
