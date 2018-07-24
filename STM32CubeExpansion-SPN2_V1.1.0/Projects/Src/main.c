/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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
#include "xnucleoihm02a1.h"
#include "params.h"
#include "example.h"
#include "example_usart.h"
#include "stm32f4xx_hal_adc.h"
#include "xnucleoihm02a1_interface.h"

// #define TEST_MOTOR	//!< Comment out this line to test the ADC

/**
  * @defgroup   MotionControl
  * @{
  */

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @}
  */ /* End of BSP */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleTypes
  * @{
  */

//#define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
#elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

/**
  * @}
  */ /* End of ExampleTypes */
	
	/* Private Variables ----------------------*/

/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue = 0;


/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
static void Error_Handler(void);
uint16_t Read_ADC(void);

// Lab 1
void initSwitchLED(void);
void switchLED(void);

// Lab 2
void initSigGenAndLED(void);
void pollForRisingEdge(void);
void initInterrupt(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void EXTI4_IRQHandler(void);

/* LAB 1 CODE */
void initSwitchLED(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Init switch */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* LED init */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void switchLED(void)
{
	GPIO_PinState readVal = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
		if (readVal == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		}
		if (readVal == GPIO_PIN_SET){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		}
}

/* LAB 2 Code */

void initSigGenAndLED(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* LED init */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* Signal Generator init */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  // GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

uint8_t lastVal = 0;

void pollForRisingEdge(void)
{
	while(1)
	{
		GPIO_PinState readVal = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		if (readVal == GPIO_PIN_RESET && readVal != lastVal){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
			lastVal = 0;
		}
		if (readVal == GPIO_PIN_SET && readVal != lastVal){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
			lastVal = 1;
		}
	}
}

void initInterruptEXTI4(void)
{
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	}
}*/

void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/* LAB 3 CODE */

void initLimitSwitches(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Switch 1 init */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* Switch 2 init */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* Init LED for testing */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void initInterruptEXTI9_5(void)
{
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}

#define MPR_4     4			  //!< 4 Motor Movements Per Revolution
#define MPR_8     8			  //!< 8 Motor Movements Per Revolution
#define DELAY_1   1000		//!< Delay time 1st option
#define DELAY_2   2500		//!< Delay time 2nd option
#define DELAY_3   10000   //!< Delay time 3rd option
uint32_t currentSpeed = 10000;
eL6470_DirId_t currentDirection = L6470_DIR_FWD_ID;
uint8_t board, device;

uint8_t id;

StepperMotorBoardHandle_t *StepperMotorBoardHandle;
MotorParameterData_t *MotorParameterDataGlobal, *MotorParameterDataSingle;

void moveMotors(void)
{
  #ifdef NUCLEO_USE_USART
  USART_Transmit(&huart2, "Initial values for registers:\n\r");
  USART_PrintAllRegisterValues();
  #endif

  /* Setup each X-NUCLEO-IHM02A1 Expansion Board ******************************/
  
  /* Get the parameters for the motor connected with the 1st stepper motor driver of the 1st stepper motor expansion board */
  MotorParameterDataGlobal = GetMotorParameterInitData();
  
  for (id = 0; id < EXPBRD_MOUNTED_NR; id++)
  {
    StepperMotorBoardHandle = BSP_GetExpansionBoardHandle(EXPBRD_ID(id));
    MotorParameterDataSingle = MotorParameterDataGlobal+(id*L6470DAISYCHAINSIZE);
    StepperMotorBoardHandle->Config(MotorParameterDataSingle);
  }
  
  #ifdef NUCLEO_USE_USART
  USART_Transmit(&huart2, "Custom values for registers:\n\r");
  USART_PrintAllRegisterValues();
  #endif
  
  /****************************************************************************/
	
	for (board = EXPBRD_ID(0); board <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); board++)
  {
    
    for (device = L6470_ID(0); device <= L6470_ID(L6470DAISYCHAINSIZE-1); device++)
    {
      /* Get the parameters for the motor connected with the actual stepper motor driver of the actual stepper motor expansion board */
      MotorParameterDataSingle = MotorParameterDataGlobal+((board*L6470DAISYCHAINSIZE)+device);
      
      /* Prepare the stepper driver to be ready to perform a command */
      StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareRun(device, currentDirection, currentSpeed);
    }
    
    StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_4 || GPIO_Pin == GPIO_PIN_6)
	{
		// Stop motors
		for (board = EXPBRD_ID(0); board <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); board++)
		{
			
			for (device = L6470_ID(0); device <= L6470_ID(L6470DAISYCHAINSIZE-1); device++)
			{
				/* Prepare the stepper driver to be ready to perform a command */
				StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareHardHiZ(device);
			}
			
			StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
		}
		
		// Wait
		for (int i = 0; i < 100; i++){}
		
		if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == 1))
		{
			// Reverse direction
			for (board = EXPBRD_ID(0); board <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); board++)
			{
				
				for (device = L6470_ID(0); device <= L6470_ID(L6470DAISYCHAINSIZE-1); device++)
				{
					/* Get the parameters for the motor connected with the actual stepper motor driver of the actual stepper motor expansion board */
					MotorParameterDataSingle = MotorParameterDataGlobal+((board*L6470DAISYCHAINSIZE)+device);
					
					if (GPIO_Pin == GPIO_PIN_4)
					{
						currentDirection = L6470_DIR_FWD_ID;
					}
					else
					{
						currentDirection = L6470_DIR_REV_ID;
					}
					
					/* Prepare the stepper driver to be ready to perform a command */
					StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareRun(device, currentDirection, currentSpeed);
				}
				
				StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
			}
		}
		else
		{
			HAL_NVIC_DisableIRQ(EXTI4_IRQn);
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		}

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	}
}

// FOR 12 BIT RESOLUTION ONLY, change threshold ADC values if changing resolution accordingly
uint32_t calcNewSpeed(uint16_t ADCVal)
{
	if (ADCVal < 2340)
	{
		currentDirection = L6470_DIR_FWD_ID;
		if (ADCVal < 585)
			return 20000;
		else if (ADCVal < 1170)
			return 13334;
		else if (ADCVal < 1755)
			return 6667;
		else
			return 0;
	}
	else
	{
		currentDirection = L6470_DIR_REV_ID;
		if (ADCVal < 2925)
			return 6667;
		else if (ADCVal < 3510)
			return 13334;
		else
			return 20000;
	}
}

/**
  * @brief The FW main module
  */
int main(void)
{
  /* NUCLEO board initialization */
	/* Init for UART, ADC, GPIO and SPI */
  NUCLEO_Board_Init();
  
  /* X-NUCLEO-IHM02A1 initialization */
  BSP_Init();
	
	// initSigGenAndLED();
	initLimitSwitches();
	
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	
	initInterruptEXTI4();
	initInterruptEXTI9_5();

	#ifdef NUCLEO_USE_USART
  /* Transmit the initial message to the PC via UART */
  USART_TxWelcomeMessage();
#endif
	
#if defined (MICROSTEPPING_MOTOR_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  MicrosteppingMotor_Example_01();
  
  /* Infinite loop */
  while (1);
#elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();
	
	/*Initialize the motor parameters */
	Motor_Param_Reg_Init();
	
	// pollForRisingEdge();
	moveMotors();
	
	MX_ADC1_Init();
  
  /* Infinite loop */
  while (1)
  {

#ifdef TEST_MOTOR		

		/* Check if any Application Command for L6470 has been entered by USART */
    USART_CheckAppCmd();
		
#else
		
		uint16_t myADCVal;
		myADCVal = Read_ADC();
		currentSpeed = calcNewSpeed(myADCVal);
		// Set new motor speed
		for (board = EXPBRD_ID(0); board <= EXPBRD_ID(EXPBRD_MOUNTED_NR-1); board++)
  {
    
    for (device = L6470_ID(0); device <= L6470_ID(L6470DAISYCHAINSIZE-1); device++)
    {
      /* Get the parameters for the motor connected with the actual stepper motor driver of the actual stepper motor expansion board */
      MotorParameterDataSingle = MotorParameterDataGlobal+((board*L6470DAISYCHAINSIZE)+device);
      
      /* Prepare the stepper driver to be ready to perform a command */
      StepperMotorBoardHandle->StepperMotorDriverHandle[device]->Command->PrepareRun(device, currentDirection, currentSpeed);
    }
    
    StepperMotorBoardHandle->Command->PerformPreparedApplicationCommand();
  }
		USART_Transmit(&huart2, " ADC Read: ");
	  USART_Transmit(&huart2, num2hex(myADCVal, WORD_F));
		USART_Transmit(&huart2, " Speed Value: ");
		USART_Transmit(&huart2, num2hex(currentSpeed, DOUBLEWORD_F));
		USART_Transmit(&huart2, " Direction: ");
		USART_Transmit(&huart2, num2hex(currentDirection, HALFBYTE_F));
		USART_Transmit(&huart2, " \n\r");
		

#endif			
  }
#endif
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @brief  This function return the ADC conversion result.
  * @retval The number into the range [0, 4095] as [0, 3.3]V.
  */
uint16_t Read_ADC(void)
{
  HAL_ADC_Start(&HADC);
  HAL_ADC_PollForConversion(&HADC, 100);
  
  return HAL_ADC_GetValue(&HADC);
}

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
