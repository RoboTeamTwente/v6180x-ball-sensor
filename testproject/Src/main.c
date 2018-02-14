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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "PuttyInterface.h"
#include "vl6180x_def.h"
#define myDev   (0x29<<1)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PuttyInterfaceTypeDef pitd;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void MyDev_SetChipEnable() {
	//STARTUP SEQUENCE:
	//Set GPIO0 to 0
	//Set GPIO0 to 1
	//Wait for a minimum of 400μs

	//uprintf("Starting chip enable\n\r");
	HAL_GPIO_WritePin(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_Pin , (GPIO_PinState)0);
	HAL_Delay(10);
    HAL_GPIO_WritePin(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_Pin , (GPIO_PinState)1);
    HAL_Delay(1);
    /* Note that as we waited  1msec we could bypass VL6180x_WaitDeviceBooted(); */
    //uprintf("Waiting for device to boot\n\r");
    //VL6180x_WaitDeviceBooted(myDev);
    //uprintf("Device booted\n\r");
}


void HandleCommand(char* input){

}

int nErr=0;
void OnErrLog(void){
    /* just log */
    nErr++;
}

void WrByte(uint8_t dev, uint16_t index, uint8_t data) {

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, myDev, (uint16_t)index, I2C_MEMADD_SIZE_16BIT, &data, 1, 10000);

	if(status != HAL_OK) {
		uprintf("WrByte failed\r\n");
	}
}

void RdByte(uint8_t dev, uint16_t index, uint8_t* data, uint8_t size) {

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, myDev, (uint16_t)index, I2C_MEMADD_SIZE_16BIT, data, size, 10000);

	if(status != HAL_OK) {
		uprintf("RdByte failed\r\n");
	}
}

void LoadSettings() {
	WrByte(myDev,0x0207, 0x01);
	WrByte(myDev,0x0208, 0x01);
	WrByte(myDev,0x0096, 0x00);
	WrByte(myDev,0x0097, 0xfd);
	WrByte(myDev,0x00e3, 0x00);
	WrByte(myDev,0x00e4, 0x04);
	WrByte(myDev,0x00e5, 0x02);
	WrByte(myDev,0x00e6, 0x01);
	WrByte(myDev,0x00e7, 0x03);
	WrByte(myDev,0x00f5, 0x02);
	WrByte(myDev,0x00d9, 0x05);
	WrByte(myDev,0x00db, 0xce);
	WrByte(myDev,0x00dc, 0x03);
	WrByte(myDev,0x00dd, 0xf8);
	WrByte(myDev,0x009f, 0x00);
	WrByte(myDev,0x00a3, 0x3c);
	WrByte(myDev,0x00b7, 0x00);
	WrByte(myDev,0x00bb, 0x3c);
	WrByte(myDev,0x00b2, 0x09);
	WrByte(myDev,0x00ca, 0x09);
	WrByte(myDev,0x0198, 0x01);
	WrByte(myDev,0x01b0, 0x17);
	WrByte(myDev,0x01ad, 0x00);
	WrByte(myDev,0x00ff, 0x05);
	WrByte(myDev,0x0100, 0x05);
	WrByte(myDev,0x0199, 0x05);
	WrByte(myDev,0x01a6, 0x1b);
	WrByte(myDev,0x01ac, 0x3e);
	WrByte(myDev,0x01a7, 0x1f);
	WrByte(myDev,0x0030, 0x00);
	// Recommended : Public registers - See data sheet for more detail
	WrByte(myDev,0x0011, 0x10); // Enables polling for ‘New Sample ready’
	// when measurement completes
	WrByte(myDev,0x010a, 0x30); // Set the averaging sample period
	// (compromise between lower noise and
	// increased execution time)
	WrByte(myDev,0x003f, 0x46); // Sets the light and dark gain (upper
	// nibble). Dark gain should not be
	// changed.
	WrByte(myDev,0x0031, 0xFF); // sets the # of range measurements after
	// which auto calibration of system is
	// performed
	WrByte(myDev,0x0040, 0x63); // Set ALS integration time to 100ms
	WrByte(myDev,0x002e, 0x01); // perform a single temperature calibration
	// of the ranging sensor
	WrByte(myDev,0x001b, 0x09); // Set default ranging inter-measurement
	// period to 100ms
	WrByte(myDev,0x003e, 0x31); // Set default ALS inter-measurement period
	// to 500ms
	WrByte(myDev,0x0014, 0x24); // Configures interrupt on ‘New Sample
	// Ready threshold event’
}

void adafruitPort()
{
	MyDev_SetChipEnable(); //toggle GPIO0 to reset device

	uint8_t reset, status, range_status, id, range;
	uint8_t val;

	//CHECK DEVICE ID
	HAL_I2C_Mem_Read(&hi2c1, myDev, (uint16_t)IDENTIFICATION_MODEL_ID, I2C_MEMADD_SIZE_16BIT, &id, 1, 10000);
	//uprintf("id: %d\r\n", id);
	if(id == 0xB4) {
		//uprintf("--> Device recognized!\n\r");
	}
	else {
		uprintf("--> Device not recognized! Exiting...\n\r");
		return;
	}

	//CHECK RESET
	HAL_I2C_Mem_Read(&hi2c1, myDev, (uint16_t)SYSTEM_FRESH_OUT_OF_RESET, I2C_MEMADD_SIZE_16BIT, &reset, 1, 10000);
	//uprintf("reset: %d\r\n", reset);

	//LOAD A BUNCH OF SETTINGS ONTO DEVICE
	LoadSettings();
	//uprintf("settings loaded\r\n");
	val = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, myDev, (uint16_t)SYSTEM_FRESH_OUT_OF_RESET, I2C_MEMADD_SIZE_16BIT, (uint8_t*)(&val), 1, 10000);

	//WAIT TILL 1ST BIT OF RANGE STATUS IS SET
	while (!( (range_status) & 0x01)){
		HAL_I2C_Mem_Read(&hi2c1, myDev, (uint16_t)RESULT_RANGE_STATUS, I2C_MEMADD_SIZE_16BIT, &range_status, 1, 10000);
	}

	//uprintf("--> range status: %d\r\n", range_status);

	// Start a range measurement
	val = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, myDev, (uint16_t)SYSRANGE_START, I2C_MEMADD_SIZE_16BIT, (uint8_t*)(&val), 1, 10000);

	HAL_I2C_Mem_Read(&hi2c1, myDev, (uint16_t)RESULT_INTERRUPT_STATUS_GPIO, I2C_MEMADD_SIZE_16BIT, &status, 1, 10000);
	//uprintf("status: %d\r\n", status);

	//WAIT TILL 2ND BIT OF RANGE STATUS IS SET
	while (! ((status) & 0x04)){
		  HAL_I2C_Mem_Read(&hi2c1, myDev, (uint16_t)RESULT_INTERRUPT_STATUS_GPIO, I2C_MEMADD_SIZE_16BIT, &status, 1, 10000);
	  }
	  //uprintf("--> status: %d\r\n", status);

	  // read range in mm
	  HAL_I2C_Mem_Read(&hi2c1, myDev, (uint16_t)RESULT_RANGE_VAL, I2C_MEMADD_SIZE_16BIT, &range, 1, 10000);
	  uprintf("range: %d\r\n", range);

	  // clear interrupt
	  val = 0x07;
	  HAL_I2C_Mem_Write(&hi2c1, myDev, (uint16_t)SYSTEM_INTERRUPT_CLEAR, I2C_MEMADD_SIZE_16BIT, (uint8_t*)(&val), 1, 10000);

	  HAL_I2C_Mem_Read(&hi2c1, myDev, (uint16_t)RESULT_RANGE_STATUS, I2C_MEMADD_SIZE_16BIT, &status, 1, 10000);
	  status = status >> 4;
	  //uprintf("status: %d\r\n", status);
}

void i2ctest()
{
	MyDev_SetChipEnable();

	uint8_t reset, status, range_status, id, range;

	RdByte(myDev, IDENTIFICATION_MODEL_ID, &id, 1);
	uprintf("size 1: id: %d\r\n", id);

	RdByte(myDev, IDENTIFICATION_MODEL_ID, &id, 2);
	uprintf("size 2: id: %d\r\n", id);

	/*RdByte(myDev, SYSTEM_FRESH_OUT_OF_RESET, &reset);
	uprintf("reset: %d\r\n", reset);

	uint8_t reset, status, range_status, id, range;

	ReadBuffer(myDev, 0x000, &id, 1);
	uprintf("id: %d\r\n", id);*/



}


/* USER CODE END 0 */

/**3
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  pitd.handle = HandleCommand;
  PuttyInterface_Init(&pitd);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_GPIO_WritePin(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_Pin , (GPIO_PinState)1);
  while (1)
  {
   /* USER CODE END WHILE */

   /* USER CODE BEGIN 3 */
	  //PuttyInterface_Update(&pitd);
	  HAL_Delay(1);
  	  adafruitPort();
	  //i2ctest();

  }

  //Sample_SimpleRanging();
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
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

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
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
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
