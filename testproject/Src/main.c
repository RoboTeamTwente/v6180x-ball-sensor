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
#include "vl6180x_api.h"
#define myDev   (0x29<<1)    // what we use as "API device
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PuttyInterfaceTypeDef pitd;
uint32_t range;
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

	uprintf("Starting chip enable\n\r");
	HAL_GPIO_WritePin(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_Pin , (GPIO_PinState)0);
	HAL_Delay(10);
    HAL_GPIO_WritePin(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_Pin , (GPIO_PinState)1);
    HAL_Delay(1);
    /* Note that as we waited  1msec we could bypass VL6180x_WaitDeviceBooted(); */
    uprintf("Waiting for device to boot\n\r");
    //VL6180x_WaitDeviceBooted(myDev);
    uprintf("Device booted\n\r");
}

void Sample_SimpleRanging(void) {
	uprintf("Starting SimpleRanging...\n\r");
	//VL6180xDev_t myDev = 0x29;
	VL6180x_RangeData_t Range;
   //MyDev_Init(myDev);           // your code init device variable
   MyDev_SetChipEnable();  // your code assert chip enable
            // your code sleep at least 1 msec
   VL6180x_InitData(myDev);
   VL6180x_Prepare(myDev);
   uprintf("Starting measurements...\n\r");
   do {
	   PuttyInterface_Update(&pitd);
       VL6180x_RangePollMeasurement(myDev, &Range);
       if (Range.errorStatus == 0 ) {

    	   uprintf("range: %ld\n\r",Range.range_mm);
    	   range = Range.range_mm;
       }
       else {
           //MyDev_ShowErr(myDev, Range.errorStatus); // your code display error code
    	   uprintf("Error status\n\r");
       }
       PuttyInterface_Update(&pitd);
   }// while (!MyDev_UserSayStop(myDev)); // your code to stop looping
   while(1);
}

void HandleCommand(char* input)
{
	if(!strcmp(input, "start"))
	{
		uprintf("started\n\r");
	}
	else if (!strcmp(input, "r"))
	{
		uprintf("%ld\n\r",range);
	}
		else if (!strcmp(input, "end"))
	{
		uprintf("ended\n\r");
	}
}

int nErr=0;
void OnErrLog(void){
    /* just log */
    nErr++;
}

void RdByte(uint8_t dev, uint16_t index, uint8_t *data) {

	    int  status;
	    uint16_t *data_write;
	    uint8_t *data_read;

	    data_write[0]=(index>>8) & 0xFF;
	    data_write[1]=index&0xFF;

	    status=HAL_I2C_Master_Transmit(&hi2c1, dev, (uint8_t*) &data_write, 2, 10000);
	    if( !status ){
	        status=HAL_I2C_Master_Receive(&hi2c1, dev, data_read, 1, 10000);
	        if( !status ){
	            *data=data_read[0];
	        	uprintf("received: %d\r\n",data_read[0]);
	        }
	        else
	        	uprintf("receive failed\r\n");
	    }
	    else
	    	uprintf("transmit failed\r\n");
}

void WrByte(uint8_t dev, uint16_t index, uint8_t data) {

	    int  status;
	    uint8_t data_write[3];

	    data_write[0] = (index >> 8) & 0xFF;; // MSB of register address
	    data_write[1] = index & 0xFF; // LSB of register address
	    data_write[2] = data & 0xFF;

	    status=HAL_I2C_Master_Transmit(&hi2c1, dev, data_write, 3, 10000);
	    if( !status ){
	            return;
	    }
	    else
	    	uprintf("transmit failed\r\n");
}

void WriteBuffer(uint8_t I2C_ADDRESS, uint8_t *aTxBuffer, uint8_t TXBUFFERSIZE)
{
    /* -> Start the transmission process */
    /* While the I2C in reception process, user can transmit data through "aTxBuffer" buffer */
    while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)myDev, (uint8_t*)aTxBuffer, (uint16_t)TXBUFFERSIZE, (uint32_t)1000)!= HAL_OK)
    {
        /*
         * Error_Handler() function is called when Timeout error occurs.
         * When Acknowledge failure occurs (Slave don't acknowledge it's address)
         * Master restarts communication
         */

        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
        {
            uprintf("In I2C::WriteBuffer -> error");
            //Error_Handler(3);
        }

    }

    /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     */
      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
      {
      }
}

void ReadBuffer(uint8_t I2C_ADDRESS, uint8_t RegAddr, uint8_t *aRxBuffer, uint8_t RXBUFFERSIZE)
{
    /* -> Lets ask for register's address */
    WriteBuffer(I2C_ADDRESS, &RegAddr, 1);

    /* -> Put I2C peripheral in reception process */
    while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)myDev, aRxBuffer, (uint16_t)RXBUFFERSIZE, (uint32_t)1000) != HAL_OK)
    {
        /* Error_Handler() function is called when Timeout error occurs.
         * When Acknowledge failure occurs (Slave don't acknowledge it's address)
         * Master restarts communication
         */
        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
        {
            uprintf( "In I2C::WriteBuffer -> error");
            //Error_Handler(4);
        }
    }

    /* -> Wait for the end of the transfer */
    /* Before starting a new communication transfer, you need to check the current
     * state of the peripheral; if it’s busy you need to wait for the end of current
     * transfer before starting a new one.
     * For simplicity reasons, this example is just waiting till the end of the
     * transfer, but application may perform other tasks while transfer operation
     * is ongoing.
     **/
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
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
/*
	int status = VL6180x_RdByte(myDev, IDENTIFICATION_MODEL_ID, &data);
	if(!status)
		uprintf("data1: %d\r\n", data);
	status = VL6180x_RdByte(myDev, SYSTEM_FRESH_OUT_OF_RESET, &data);
		if(!status)
			uprintf("data2: %d\r\n", data);
	status = VL6180x_Prepare(myDev);
	if(!status)
			uprintf("init success\r\n");*/
	uint8_t data2,data3 = 1;

	//int status = HAL_I2C_Mem_Read(&hi2c1, (0x29<<1), IDENTIFICATION_MODEL_ID, I2C_MEMADD_SIZE_8BIT, &data, 2, 10000);

	//if(status == HAL_OK)
	    //{
		//uprintf("data1: %d\r\n", data);
	    //}
/*	RdByte(myDev, IDENTIFICATION_MODEL_ID, &data3);
	uprintf("data2: %d\r\n", data2);
	RdByte(myDev, SYSTEM_FRESH_OUT_OF_RESET, &data2);
	uprintf("data3: %d\r\n", data3);*/

	uint8_t reset, status, range_status, id, range;

	RdByte(myDev, IDENTIFICATION_MODEL_ID, &id);
	uprintf("id: %d\r\n", id);

	RdByte(myDev, SYSTEM_FRESH_OUT_OF_RESET, &reset);
	uprintf("reset: %d\r\n", reset);

	LoadSettings();
	uprintf("settings loaded\r\n");
	WrByte(myDev, SYSTEM_FRESH_OUT_OF_RESET, 0x00);
	RdByte(myDev, RESULT_RANGE_STATUS, &range_status);
	uprintf("range status: %d\r\n", range_status);

	while (!( (range_status) & 0x01)){
		RdByte(myDev, RESULT_RANGE_STATUS, &range_status);
	}

	uprintf("--> range status: %d\r\n", range_status);

	  // Start a range measurement
	WrByte(myDev, SYSRANGE_START, 0x01);

	RdByte(myDev, RESULT_INTERRUPT_STATUS_GPIO, &status);
	//uprintf("status: %d\r\n", status);

	  // Poll until bit 2 is set
	  while (! ((status) & 0x04)){
		  RdByte(myDev, RESULT_INTERRUPT_STATUS_GPIO, &status);
	  }
	  uprintf("--> status: %d\r\n", status);

	  // read range in mm
	  RdByte(myDev, RESULT_RANGE_VAL, &range);
	  uprintf("range: %d\r\n", range);

	  // clear interrupt
	  WrByte(myDev, SYSTEM_INTERRUPT_CLEAR, 0x07);

	  RdByte(myDev, RESULT_RANGE_STATUS, &status);
	  status = status >> 4;
	  uprintf("status: %d\r\n", range);
}

void i2ctest()
{
/*	uint8_t reset, status, range_status, id, range;

	RdByte(myDev, IDENTIFICATION_MODEL_ID, &id);
	uprintf("id: %d\r\n", id);

	RdByte(myDev, SYSTEM_FRESH_OUT_OF_RESET, &reset);
	uprintf("reset: %d\r\n", reset);*/

	uint8_t reset, status, range_status, id, range;

	ReadBuffer(myDev, 0x000, &id, 1);
	uprintf("id: %d\r\n", id);

}


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
  HAL_GPIO_WritePin(CHIP_ENABLE_GPIO_Port, CHIP_ENABLE_Pin , (GPIO_PinState)1);
  //while (1)
  //{
   /* USER CODE END WHILE */

   /* USER CODE BEGIN 3 */
	  //PuttyInterface_Update(&pitd);
	  //HAL_Delay(1);


  //}
  i2ctest();
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
