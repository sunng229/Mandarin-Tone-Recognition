/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "my_mfcc.h"
#include "svm.h"
#include <stdio.h>
#include <stdlib.h>

#define ARM_MATH_CM7
//#define __FPU_PRESENT 1

#define ADC_BUF_SIZE 1024*16*3   // 16kHz x 3s
uint16_t ADC_buff[ADC_BUF_SIZE];
float ADC_buff_norm[ADC_BUF_SIZE];
float *voice_buff;
int record_done=0;
uint8_t name[20];//name of the file
// USB FatFs
extern ApplicationTypeDef Appli_state;
void writeFile(char filename[],  uint16_t *vec, int size);
int loadSVMParams(char filename[], struct svm_model* model);
float compute_power(uint16_t size_in, float *buff_in);
float *removeSilence(uint16_t size_in, float *buff_in, uint16_t *size_out);

// MFCC
#define pi 3.1415
#define FFTSize 512
#define n_mfcc 13
uint32_t n_mels = 20;

uint32_t sample_rate = 16000;
uint32_t freq_min = 0;
uint32_t freq_max = 8000;

//extern float32_t debug[];
extern float32_t debug_feature[];
//extern float32_t win_ref[];
//extern float32_t power_spec_ref[];

float32_t complex_buff[FFTSize*2];
float32_t power_spec_buff[FFTSize/2+1];

float32_t filt_spec_buff[20];
float32_t dct_spec_buff[20];


//extern float32_t debug_mfcc_scaled1;
//extern float32_t debug_mfcc_scaled2;
//extern float32_t debug_mfcc_scaled3;
//extern float32_t debug_mfcc_scaled4;
//extern float32_t debug_mfcc_scaled5;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float *removeSilence(uint16_t size_in, float *buff_in, uint16_t *size_out)
{
	int slide_start, slide_end, start, end;
	float max_val=0, max_idx, avg_power,slide_power, threshold, *buff_out;

	for(int i=0; i<size_in; i++){
		if(fabsf(buff_in[i]) > max_val){
			max_idx = i;
			max_val = fabsf(buff_in[i]);
		}
	}

    slide_start = max_idx - 256;
    slide_end = max_idx + 256;

    avg_power = compute_power(512, &buff_in[slide_start]);
    threshold = 0.2 * avg_power;

    for(int i=max_idx; i<(size_in-256); i++){
        slide_start = i - 256;
        slide_end = i + 256;

        slide_power = compute_power(512, &buff_in[slide_start]);

		if (slide_power < threshold){
			end = slide_end;
			break;
		}
    }

    for(int i=max_idx; i>256; i-- ){
        slide_start = i - 256;
        slide_end = i + 256;

        slide_power = compute_power(512, &buff_in[slide_start]);
        if (slide_power < threshold){
            start = slide_start;
            break;
        }
    }


	// copy voice(without silence) to buff_out
	*size_out = end - start;
	if(*size_out > 3*FFTSize){
		// allocate memory for buff_out
		buff_out = (float *)malloc(sizeof(float)*(*size_out));
		for(int i=0; i<*size_out; i++){
			buff_out[i] = buff_in[i+start];
		}
		return buff_out;
	} else {
		 Error_Handler();
	}


}
float compute_power(uint16_t size_in, float *buff_in)
{
	float acc=0;
	for(int i=0; i<size_in; i++){
		acc += buff_in[i] * buff_in[i];
	}
	return sqrtf(acc)/size_in;
}


void normalizeSignal(int size, uint16_t *buff_in, float *buff_out);
void normalizeSignal(int size, uint16_t *buff_in, float *buff_out)
{
	int i;
	double mean_val, std_val, tmp;

	mean_val = 24632.872463903754;
	std_val = 2689.7324473383696;

	/** normalization **/
	for(i=0; i<size; i++){
		buff_out[i] = (buff_in[i] - mean_val)/std_val;
	}

}

//void scale_feature()

float freqToMel(float freq);
float melToFreq(float mel);
void powerToDB(uint32_t size, float32_t *real_buff);
void convertToComplex(int size, const float32_t *real_buff, float32_t *complex_buff);
void powerSpectrum(uint32_t size, const float32_t *complex_buff, float32_t *real_buff);
void filteredSpectrum( uint32_t sr, uint32_t n_fft, uint32_t n_mels, float32_t freq_min,
						float32_t freq_max, const float32_t *power_spec_buff, float32_t *filtered_spec);
void normalizeFeature(int size, float *feature, float *mean, float *scale);
/**
 * convert real array to complex array for cFFT
 * example:
 * input real_buff=[1.1, 1.2, 1.3, 1.4, 1.5]
 * output complex_buff=[1.1, 0, 1.2, 0, 1.3, 0, 1.4, 0, 1.5, 0]
 */
void convertToComplex(int size, const float32_t *real_buff, float32_t *complex_buff)
{
	  for(int i = 0; i < size; i++){
		  complex_buff[2*i] = real_buff[i];
		  complex_buff[2*i + 1] = 0;
	  }
}

/**
 * compute power spectrum from FFT complex buff
 * |complex_buff|^2
 * since FFT complex buff is symmetric, only half of this buff is needed
 */
void powerSpectrum(uint32_t size, const float32_t *complex_buff, float32_t *real_buff)
{
	  for(int i=0; i < (size/2+1); i++){
		  real_buff[i] = powf(fabsf(complex_buff[2*i]), 2) + powf(fabsf(complex_buff[2*i + 1]), 2);
	  }
}

/**
 * log transformation
 * real_buff = log(real_buff)
 * in-place operation, real_buff will be updated
 * **/
void powerToDB(uint32_t size, float32_t *real_buff){
	for(int i=0; i<size; i++){
		if(real_buff[i] < 0){
			exit(0); // power spectrum must be non-negative
		}
		real_buff[i] = 10*log10f(real_buff[i]);
	}
}

/**
 * Discrete Cosine Transform (DCT type-II)
 * More details refer to https://docs.scipy.org/doc/scipy/reference/generated/scipy.fftpack.dct.html
 */
void dctTransform(uint32_t size, float32_t *buff_in, float32_t *buff_out)
{
	float acc, scale;

	for (int i = 0; i < size; i++) {
		acc = 0.0;
		for (int j = 0; j < size; j++) {
			acc += buff_in[j] * cosf(pi * (j + 0.5) * i / size);
		}
		scale = (i==0) ? sqrtf(1.0/(4.0 * size)) * 2: sqrtf(1.0/(2.0 *size)) * 2;
		buff_out[i] = acc * scale;
	}
}

/**
 * triangular filter band
 */
void filteredSpectrum( uint32_t sr, uint32_t n_fft, uint32_t n_mels, float32_t freq_min,
						float32_t freq_max, const float32_t *power_spec_buff, float32_t *filtered_spec)
{
	float fmin_mel, fmax_mel, mel_step, linear_freq_step, inner_sum;

	int i=0, jth_filter=0;

	float filter[n_fft/2+1];  			// size 257
	float mels[n_mels+2];        		// size 22
	float mels_f[n_mels+2];        	// size 22
	float linar_freq[n_fft/2+1];
	float upper[n_fft/2+1], lower[n_fft/2+1];
	float enorm[n_mels];                // size 20

	fmin_mel = freqToMel(freq_min); 		// 0.0
	fmax_mel = freqToMel(freq_max); 		// 2840.0377

	/** Center freqs of each FFT bin **/
	linear_freq_step =  (sr/2.0 - freq_min)/(n_fft/2);
	for(i=0; i<(n_fft/2+1); i++){
		linar_freq[i] = i*linear_freq_step;
	}

	/** 'Center freqs' of mel bands - uniformly spaced between limits **/
	mel_step =  (fmax_mel - fmin_mel)/(n_mels + 1);
	for(i=0; i<(n_mels+2); i++){
		mels[i] = i*mel_step;
	}
	for(i=0; i<(n_mels+2); i++){
		mels_f[i] = melToFreq(mels[i]);
	}

	/** Slaney-style mel is scaled to be approx constant energy per channel **/
	for(i=0; i<n_mels; i++){
		enorm[i] = 2.0 / (mels_f[i+2] - mels_f[i]);
	}

	/**
	 * iterate all filters
	 */
	for(jth_filter=0; jth_filter<n_mels; jth_filter++){

		/** update upper/lower slop of filter bins**/
		for(i=0; i<(n_fft/2+1); i++){
			lower[i] = (linar_freq[i] - mels_f[jth_filter]) / (mels_f[jth_filter+1]-mels_f[jth_filter]);
		}
		for(i=0; i<(n_fft/2+1); i++){
			upper[i] = (mels_f[jth_filter+2] - linar_freq[i])/ (mels_f[jth_filter+2]-mels_f[jth_filter+1]);
		}

		/** compute filter matrix **/
		for(i=0; i<(n_fft/2+1); i++){
			filter[i] = fmaxf(0, fminf(upper[i], lower[i]));
		}

		/** normalize filter matrix **/
		for(i=0; i<(n_fft/2+1); i++){
			filter[i] *=  enorm[jth_filter];
		}

		/** compute filtered spectrum **/
		inner_sum = 0;
		for(i=0; i<(n_fft/2+1); i++){
			if(filter[i] == 0){
				continue;        //skip zero multiplication
			} else {
				inner_sum += filter[i] * power_spec_buff[i];
			}
		}
		filtered_spec[jth_filter] = inner_sum;
	}

}



float freqToMel(float freq)
{
	return 1127.01048 * log(1 + freq / 700.0);
}


float melToFreq(float mel)
{
	return 700 * (exp(mel / 1127.01048) - 1);
}

/**
 * add hamming window to input signal
 * in-place operation
 */
void hammingWindow(int size, float *real_buff)
{
	float hamming[size];
	for (int i = 0; i < size; i++){
		hamming[i] = 0.54 - 0.46 * cos(2 * pi * i / size);
	}

	for (int i = 0; i < size; i++){
		real_buff[i] *= hamming[i];
	}
}



void normalizeFeature(int size, float *feature, float *scaler_min, float *scaler_max)
{

// (X_train.to_numpy()[0] - scaler.data_min_)/(scaler.data_max_ - scaler.data_min_)
	for(int i=0; i<size; i++){
		feature[i] = (feature[i] - scaler_min[i])/(scaler_max[i] - scaler_min[i]);
	}
}
/**
 * compare two vectors
 * if their RMSE value is larger than tolerance, return 1
 * else return 0
 */
int rmse_check(float *a, float *b, int size, float tol)
{
	float error = 0;
	for(int i=0; i<size; i++){
		error += powf(a[i] - b[i], 2);
	}
	error = sqrtf(error/size);

	if(error > tol){
		return 1;
	}
	else {
		return 0;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK){
	  Error_Handler();
  }
  HAL_TIM_Base_Start(&htim1);

  // load SVM parameters from USB
	svm_model model;
	while(1){
		MX_USB_HOST_Process();

		if(Appli_state == APPLICATION_READY) {

			// read testing voice
		    voice_buff = read_txt("f4.txt", ADC_BUF_SIZE);
		    for(int i=0; i<ADC_BUF_SIZE; i++) {
		    	ADC_buff[i] = voice_buff[i];
		    }
		    free(voice_buff);

		    // load SVM parameters
			loadSVMParams("svmmodel.txt", &model);
			break; 		// load parameter successfully, break while loop

		}
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

	/**  Normalize signal, same range to training set **/
	normalizeSignal(ADC_BUF_SIZE, ADC_buff, ADC_buff_norm);

	/** Remove silence **/
	float *voice_signal;
	uint16_t size_out;
	voice_signal = removeSilence(ADC_BUF_SIZE, ADC_buff_norm, &size_out);   // not robust


	/** Framing **/
	int overlap_size = 128;
	int n_frames = 1 + (size_out - FFTSize) / overlap_size;
	if(n_frames < 3){
		Error_Handler();  // we need concatenate 3 frames' MFCC as input features
	}

	int n_res=n_frames/3-1, res;
	int vote_res[model.n_class];
	for(int i=0; i<model.n_class; i++)
		vote_res[i] = 0;

	float mfcc_feature[3*n_mfcc];
	float frame[FFTSize];

	arm_cfft_instance_f32 fft_handler;
	arm_cfft_init_f32(&fft_handler, 512);


	for(int k=0; k<n_res; k++) {

		for(int i=0; i<3; i++) {

			/** get a frame**/
			for(int j=0; j<FFTSize; j++){
				frame[j] = voice_signal[k*(FFTSize-overlap_size) + i*(FFTSize-overlap_size) + j];
			}

			/** add hamming window to a frame**/
			hammingWindow(FFTSize, frame);
	//		rmse_check(debug, win_ref, FFTSize, 0.01);

			/** FFT **/
			convertToComplex(FFTSize, frame, complex_buff);

			arm_cfft_f32(&fft_handler, complex_buff, 0, 1);

			/** compute power spectrum**/
			powerSpectrum(FFTSize, complex_buff, power_spec_buff); // power_spec_buff size: FFTSize/2 + 1 = 257

			/** compute filter bank, and use it to get filtered spectrum **/
			filteredSpectrum(sample_rate, FFTSize, n_mels, freq_min, freq_max, power_spec_buff, filt_spec_buff);

			/** log transformation **/
			powerToDB(n_mels, filt_spec_buff);

			/**  Discrete Cosine Transform (DCT type-II) **/
			dctTransform(n_mels, filt_spec_buff, dct_spec_buff);

			/**  Concate 3 frame's MFCC features**/
			for(int j=0; j<3*n_mfcc; j++){
				mfcc_feature[i * n_mfcc + j] = dct_spec_buff[j];
			}

		}

		/**  normalize MFCC feature **/
		normalizeFeature(model.n_feature, mfcc_feature, model.scaler_min, model.scaler_max);

//				rmse_check(power_spec_buff, power_spec_ref, FFTSize/2+1, 0.5);
		/*************Write your code start***************/

		// refer to svm.c in this base project
		// write your implementation in function svmPredict()

		// only for your check
		 res = svmPredict(debug_mfcc_scaled1, &model);
		 printf("Prediction result: %d \n", res);   // 0
		 res = svmPredict(debug_mfcc_scaled2, &model);
		 printf("Prediction result: %d \n", res);   // 1
		 res = svmPredict(debug_mfcc_scaled3, &model);
		 printf("Prediction result: %d \n", res);   // 2
		 res = svmPredict(debug_mfcc_scaled4, &model);
		 printf("Prediction result: %d \n", res);   // 3
		 res = svmPredict(debug_mfcc_scaled5, &model);
		 printf("Prediction result: %d \n", res);   // 4

		 // testing voice result
		 res = svmPredict(mfcc_feature, &model);
		 vote_res[res]++;

		/*************Write your code End***************/
  }

	int vote_max_idx = 0;
	for(int i=1;i<model.n_class;i++)
		if(vote_res[i] > vote_res[vote_max_idx])
			vote_max_idx = i;

	/*************Write your code start***************/

	// SWV print final results / Or check final results on live expression window

	/*************Write your code End***************/

	while(1);


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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 18;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4687-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(HAL_ADC_Stop_DMA(&hadc1) != HAL_OK){
		Error_Handler();
	}
	record_done = 1;
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
