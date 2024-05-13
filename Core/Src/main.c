/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	SENSOR_OFFSET=0,//sensor offset
	USER_OFFSET 		// user_offset
}sensor_offset;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADXL345_add (0x53 << 1)

#define POWER_CTL_REG		  0x2D
#define DATA_FORMAT_REG   0x31

#define XYZ_OFFSET_ADD    0x1E
#define XYZ_DATA_ADD      0x32
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart_send_data[32];
uint8_t rx_data;
int16_t xyz_global[3];
int32_t first_lsb_val[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void uart_send(uint8_t uart_num, char *fmt,...);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
extern HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
void i2c_check_address();

//ADXL345 function
void ADXL345_Read_offset();
void ADXL345_Read_XYZ();
void ADXL345_init(sensor_offset user_setting);


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1){
		uart_send(1,"rx : %c 0x%02x\n",rx_data,rx_data);
		HAL_UART_Receive_DMA(&huart1,&rx_data,1);
	}
  UNUSED(huart);
}

void i2c_check_address(){
	for(uint8_t i=0;i<128;i++){
		if(HAL_I2C_IsDeviceReady(&hi2c1,i<<1,1,10)==HAL_OK){
			uart_send(1,"i2c address is 0x%02X\n",i);
		}
	}
}

void ADXL345_Read_offset(){

	uint8_t _3bit_data[3];
	if(HAL_I2C_IsDeviceReady(&hi2c1,ADXL345_add,1,10)!=HAL_OK){
		uart_send(1,"%d address, invalid address or invalid connected!\n",ADXL345_add);
		return;
	}
	HAL_I2C_Mem_Read(&hi2c1,ADXL345_add,XYZ_OFFSET_ADD,1,&_3bit_data[0],3,10);
	uart_send(1,"offset val\n");
	uart_send(1,"  X   Y   Z\n");
	uart_send(1,"0x%02X 0x%02X 0x%02X\n",_3bit_data[0],_3bit_data[1],_3bit_data[2]);
}

void ADXL345_Read_XYZ(){
	uint8_t xyz_data[6];
	uint8_t j=0;
	if(HAL_I2C_IsDeviceReady(&hi2c1,ADXL345_add,1,10)!=HAL_OK){
		uart_send(1,"%d address, invalid address or invalid connected!\n",ADXL345_add);
		return;
	}
	HAL_I2C_Mem_Read(&hi2c1,ADXL345_add,XYZ_DATA_ADD,1,xyz_data,6,1000);
	for (uint8_t i = 0;i<5;i+=2){
		xyz_global[j++] = (xyz_data[i+1] << 8) | xyz_data[i];
	}

	long double xyz_val[3];
	int16_t xyz_int[3];
	uint8_t xyz_char[3]="xyz";

	for(uint8_t i=0;i<3;i++){
		xyz_val[i] = (int16_t) (xyz_global[i] - first_lsb_val[i]);

		xyz_val[i] = (xyz_val[i]*4)/1000;
		xyz_int[i] = xyz_val[i]*1000; //mg
		if((xyz_int[i]<1000 && xyz_int[i]>0)||
			 (xyz_int[i]>-1000 && xyz_int[i]<0)){
			uart_send(1,"%c:%dmg ",xyz_char[i],xyz_int[i]);
		}
		else if((xyz_int[i]>=1000)||
				    (xyz_int[i]<=-1000)||
						(xyz_int[i]==0)){
			if(xyz_int[i]>=0){
				uart_send(1,"%c:%d.%dg ",xyz_char[i],xyz_int[i]/1000,xyz_int[i]/100-(xyz_int[i]/1000)*10);
			}
			else if(xyz_int[i]<0){
				uart_send(1,"%c:%d.%dg ",xyz_char[i],xyz_int[i]/1000,(xyz_int[i]/100-(xyz_int[i]/1000)*10)*-1);
			}
		}
	}
	uart_send(1,"\n\n\n");
}

void ADXL345_init(sensor_offset user_setting){
	first_lsb_val[0]=0;
	first_lsb_val[1]=0;
	first_lsb_val[2]=0;

	//0 is sensor first setting offset,
	uint8_t power_ctl = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, ADXL345_add, POWER_CTL_REG, I2C_MEMADD_SIZE_8BIT, &power_ctl, 1, 1000);

	uint8_t data_format = 0x0B;
  HAL_I2C_Mem_Write(&hi2c1, ADXL345_add, DATA_FORMAT_REG, I2C_MEMADD_SIZE_8BIT, &data_format, 1, 1000);

  uint8_t average_number = 10;//Data Average Sensor Detection Count
  if(USER_OFFSET==user_setting){
  	for(uint8_t j=0;j<average_number;j++){
  		ADXL345_Read_XYZ();
			for(uint8_t i=0;i<3;i++){
				first_lsb_val[i]+=xyz_global[i];
			}
  	}
  	for(uint8_t i=0;i<3;i++){
  		first_lsb_val[i]/=average_number;
  	}
  }
  else if(SENSOR_OFFSET==user_setting){
  	for(uint8_t i=0;i<3;i++){
  		first_lsb_val[i]=0;
  		HAL_I2C_Mem_Write(&hi2c1, ADXL345_add, 0x1D + i, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&first_lsb_val[i], 1, 1000);
  	}
  }
}


void uart_send(uint8_t uart_num, char *fmt,...){
	va_list arg;
	va_start(arg,fmt);
	vsnprintf((char *)&uart_send_data[0],32,fmt,arg);

	if(uart_num==1){
		HAL_UART_Transmit(&huart1,uart_send_data,32,10);
	}

	memset(uart_send_data,0,32);
	va_end(arg);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  UART_Start_Receive_DMA(&huart1,&rx_data,1);

  i2c_check_address();
  HAL_Delay(10);
  ADXL345_init(SENSOR_OFFSET);
  HAL_Delay(10);
	ADXL345_Read_offset();

	uint32_t pretime=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if(HAL_GetTick()-pretime>500){
  		ADXL345_Read_XYZ();
  		pretime=HAL_GetTick();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
