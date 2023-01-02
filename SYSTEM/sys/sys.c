#include "sys.h"  
#include "dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
//STM32VN
//////////////////////////////////////////////////////////////////////////////////  

////////////////////////////////////////////////////////////////////////
//một số chức năng


void pinMode(uint32_t PINx,uint32_t MODE)
{  
   //uint32_t pinpos=0,pos=0,curpin=0;
	
	 GPIO_InitTypeDef GPIO_InitStructure;
	 GPIO_TypeDef *GPIOx = g_APinDescription[PINx].pPort;
	 uint16_t BITx = g_APinDescription[PINx].ulPin;
	 uint32_t CLK = g_APinDescription[PINx].ulPeripheral;
	
	if (CLK == RCC_CLK_GPIOA) {
		// 
		__HAL_RCC_GPIOA_CLK_ENABLE();

	} else if (CLK == RCC_CLK_GPIOB) {
		// 
		__HAL_RCC_GPIOB_CLK_ENABLE();

	} else if (CLK == RCC_CLK_GPIOC) {
		// 
		__HAL_RCC_GPIOC_CLK_ENABLE();

	}else if (CLK == RCC_CLK_GPIOD) {
		__HAL_RCC_GPIOD_CLK_ENABLE();

	}else if (CLK == RCC_CLK_GPIOE) {
		__HAL_RCC_GPIOE_CLK_ENABLE();

	}else if (CLK == RCC_CLK_GPIOF) {
		#ifdef GPIOF
		__HAL_RCC_GPIOF_CLK_ENABLE();
		#endif

	}else if (CLK == RCC_CLK_GPIOG) {
		#ifdef GPIOG
		__HAL_RCC_GPIOG_CLK_ENABLE();
		#endif

	}else if (CLK == RCC_CLK_GPIOH) {
		#ifdef GPIOH
		__HAL_RCC_GPIOH_CLK_ENABLE();
		#endif

	}else if (CLK == RCC_CLK_GPIOI) {
		#ifdef GPIOI
		__HAL_RCC_GPIOI_CLK_ENABLE();
		#endif

	}

  //////////////////////////////////////////////////////

  GPIO_InitStructure.Pin = BITx;
	
	if (MODE == INPUT) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_NOPULL;

	} else if (MODE == INPUT_PULLUP) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_PULLUP;

	} else if (MODE == INPUT_PULLDOWN) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;

	}else if (MODE == OUTPUT) {
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

	}else if (MODE == OUTPUT_OD) {
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

	} else if (MODE == AF_OD) {
		//
		GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStructure.Pull     = GPIO_NOPULL;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

	}else if (MODE == AF_PP) {
		// 
		 GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
		 GPIO_InitStructure.Pull     = GPIO_NOPULL;
		 GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;		
		 //GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
		 //GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;

	}else if (MODE == AN_INPUT) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;

	}else if (MODE == CHANGE) {
		// 
		 GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
		 GPIO_InitStructure.Pull = GPIO_NOPULL;

	}else if (MODE == FALLING) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStructure.Pull = GPIO_PULLUP;

	}else if (MODE == RISING) {
		// 
		GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
		 GPIO_InitStructure.Pull = GPIO_PULLDOWN;

	}
	HAL_GPIO_Init(GPIOx, &GPIO_InitStructure); 
	
        
} 

//void pinModeEX(uint32_t PINx,uint32_t MODE,uint32_t Alternate)
//{  
//   //uint32_t pinpos=0,pos=0,curpin=0;
//	
//	 GPIO_InitTypeDef GPIO_InitStructure;
//	 GPIO_TypeDef *GPIOx = g_APinDescription[PINx].pPort;
//	 uint16_t BITx = g_APinDescription[PINx].ulPin;
//	 uint32_t CLK = g_APinDescription[PINx].ulPeripheral;
//	
//	if (CLK == RCC_CLK_GPIOA) {
//		// 
//		__HAL_RCC_GPIOA_CLK_ENABLE();

//	} else if (CLK == RCC_CLK_GPIOB) {
//		// 
//		__HAL_RCC_GPIOB_CLK_ENABLE();

//	} else if (CLK == RCC_CLK_GPIOC) {
//		// 
//		__HAL_RCC_GPIOC_CLK_ENABLE();

//	}else if (CLK == RCC_CLK_GPIOD) {
//		__HAL_RCC_GPIOD_CLK_ENABLE();

//	}else if (CLK == RCC_CLK_GPIOE) {
//		__HAL_RCC_GPIOE_CLK_ENABLE();

//	}else if (CLK == RCC_CLK_GPIOF) {
//		#ifdef GPIOF
//		__HAL_RCC_GPIOF_CLK_ENABLE();
//		#endif

//	}else if (CLK == RCC_CLK_GPIOG) {
//		#ifdef GPIOG
//		__HAL_RCC_GPIOG_CLK_ENABLE();
//		#endif

//	}else if (CLK == RCC_CLK_GPIOH) {
//		#ifdef GPIOH
//		__HAL_RCC_GPIOH_CLK_ENABLE();
//		#endif

//	}else if (CLK == RCC_CLK_GPIOI) {
//		#ifdef GPIOI
//		__HAL_RCC_GPIOI_CLK_ENABLE();
//		#endif

//	}

//  //////////////////////////////////////////////////////

//  GPIO_InitStructure.Pin = BITx;
//	GPIO_InitStructure.Alternate = Alternate;
//	
//	if (MODE == INPUT) {
//		// 
//		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//		GPIO_InitStructure.Pull = GPIO_NOPULL;

//	} else if (MODE == INPUT_PULLUP) {
//		// 
//		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//		GPIO_InitStructure.Pull = GPIO_PULLUP;

//	} else if (MODE == INPUT_PULLDOWN) {
//		// 
//		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
//		GPIO_InitStructure.Pull = GPIO_PULLDOWN;

//	}else if (MODE == OUTPUT) {
//		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

//	}else if (MODE == OUTPUT_OD) {
//		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
//    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

//	} else if (MODE == AF_OD) {
//		//
//		GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
//		GPIO_InitStructure.Pull     = GPIO_NOPULL;
//		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

//	}else if (MODE == AF_PP) {
//		// 
//		 GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//		 GPIO_InitStructure.Pull     = GPIO_NOPULL;
//		 GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;		
//		 //GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
//		 //GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;

//	}else if (MODE == AN_INPUT) {
//		// 
//		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
//		GPIO_InitStructure.Pull = GPIO_PULLDOWN;

//	}else if (MODE == CHANGE) {
//		// 
//		 GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
//		 GPIO_InitStructure.Pull = GPIO_NOPULL;

//	}else if (MODE == FALLING) {
//		// 
//		GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
//		GPIO_InitStructure.Pull = GPIO_PULLUP;

//	}else if (MODE == RISING) {
//		// 
//		GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
//		 GPIO_InitStructure.Pull = GPIO_PULLDOWN;

//	}
//	HAL_GPIO_Init(GPIOx, &GPIO_InitStructure); 
//	
//        
//} 

void digitalWrite( uint32_t ulPin, uint8_t ulVal )
{
  /* Handle */

//  if ( ulVal == HIGH )
//  {
//    g_APinDescription[ulPin].pPort->BSRR = g_APinDescription[ulPin].ulPin;
//  }
//  else
//  {
//    g_APinDescription[ulPin].pPort->BSRR = (uint32_t)g_APinDescription[ulPin].ulPin << 16;
//  }
	   ulVal=!ulVal;/* "set" bits are lower than "reset" bits  */
  g_APinDescription[ulPin].pPort->BSRR = (uint32_t)g_APinDescription[ulPin].ulPin << (16*ulVal);
}

int digitalRead( uint32_t ulPin )
{
	
	return HAL_GPIO_ReadPin(g_APinDescription[ulPin].pPort,g_APinDescription[ulPin].ulPin) ;
	//return digitalPinToPort(ulPin)->IDR & (g_APinDescription[ulPin].ulPin);
}
//////////////////////////////////
// Deprecated these functions as they are not part of the standard Arduino API
void togglePin(uint8_t pin) {
    
g_APinDescription[pin].pPort->ODR = g_APinDescription[pin].pPort->ODR ^ (digitalPinToBitMask(pin));
   
}



uint8_t isButtonPressed(uint8_t pin, uint32_t pressedLevel) {
    if (digitalRead(pin) == pressedLevel) {
        delay(1);//BUTTON_DEBOUNCE_DELAY
        while (digitalRead(pin) == pressedLevel);
        return true;
    }
    return false;
}

uint8_t waitForButtonPress(uint8_t pin, uint32_t pressedLevel,uint32_t timeout) {
    uint32_t start = millis();
    uint32_t time;
    if (timeout == 0) {
        while (!isButtonPressed(pin, pressedLevel));
        return true;
    }
    do {
        time = millis();
        /* properly handle wrap-around */
        if ((start > time && time + (0xffffffffU - start) > timeout) ||
            time - start > timeout) {
            return false;
        }
    } while (!isButtonPressed(pin, pressedLevel));
    return true;
}
///////////////////////////////////

unsigned long pulseIn( uint32_t PINx, uint8_t state, unsigned long timeout)
{
	
uint32_t start =0 , time=0;	
unsigned long startMicros = micros();


//if (state)	GPIO_Set(GPIOx,PINx,GPIO_MODE_IN,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PD);
//	else 			GPIO_Set(GPIOx,PINx,GPIO_MODE_IN,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
	
	if (state)	pinMode(PINx,INPUT_PULLDOWN);
	else 			pinMode(PINx,INPUT_PULLUP);
	
//// wait for any previous pulse to end

	while (digitalRead(PINx) == 1) {
	if (micros() - startMicros > timeout)
	return 0;
	}
	//// wait for the pulse to start
	while (digitalRead(PINx) == 0) {
		if (timeout-- == 0x00) {
			return 0;
		}
	}
	
	/* Start time */
	start = micros();time=0;
	//// wait for the pulse to stop
	/* Wait till signal is low */
	while (digitalRead(PINx) == 1) {		
		if (micros() - startMicros > timeout) return 0;
		time++;delay_us(1);
	}
	
return micros() - start;//time;
} 
//////////////////////////////////////////
extern uint8_t pinEnabled[];
void pwmMode( uint32_t ulPin, uint32_t pwmFre, uint32_t pwmMode)
{
	//GPIO_InitTypeDef GPIO_InitStructure;
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_HandleTypeDef  TIMx;
	TIM_OC_InitTypeDef   TIM_OCInitStructure;
	TIM_MasterConfigTypeDef sMasterConfig;
        
//  
//  GPIO_TypeDef *gpio_port = g_APinDescription[ulPin].pPort;
//  uint16_t gpio_pin = g_APinDescription[ulPin].ulPin;
//	

	if ( g_APinDescription[ulPin].ulTimerPeripheral == NULL)
	{
		/*
			// Defaults to digital write
			pinMode(ulPin, OUTPUT);
			ulValue = mapResolution(ulValue, _writeResolution, 8);
			if (ulValue < 128)
			  digitalWrite(ulPin, LOW);
			else
			  digitalWrite(ulPin, HIGH);
		
			return;
			*/
	}
	

	 if (!pinEnabled[ulPin]) {
			
			
			PWM_Manager.curPin = ulPin;
	
			if (pwmMode == PWM_8_BIT){
				
				PWM_Manager.arr = 255;
				}
			else if (pwmMode == PWM_12_BIT) {
				PWM_Manager.arr = 4095;
				}
			else
				{

			}
				
			
			PWM_Manager.presc =(SystemCoreClock / (pwmFre * PWM_Manager.arr) ) - 1;

			if (PWM_Manager.arr == 4095 && (pwmFre * PWM_Manager.arr) > 72000000)
				PWM_Manager.presc = 0;

		    
			if (PWM_Manager.arr == 255 && (pwmFre * PWM_Manager.arr) > 72000000)
							PWM_Manager.presc = 0;
				
			// Setup PWM for this pin
//                     RCC_APB2PeriphClockCmd(g_APinDescription[ulPin].ulPeripheral,ENABLE);

//                        GPIO_InitStructure.GPIO_Pin = gpio_pin;
//			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//			GPIO_Init(g_APinDescription[ulPin].pPort, &GPIO_InitStructure); 
			
//		GPIO_InitTypeDef GPIO_InitStructure;
//		GPIO_TypeDef *gpio_port = g_APinDescription[ulPin].pPort;
//		uint16_t gpio_pin = g_APinDescription[ulPin].ulPin;
	
		__HAL_RCC_AFIO_CLK_ENABLE();
		pinMode(ulPin, AF_PP);//cap truoc RCC
			
		// TIM clock enable
    if(g_APinDescription[ulPin].ulTimerPeripheral == TIM1)
		{
			//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
			//TIMx.Instance = TIM1;
			//GPIO_InitStructure.Alternate = GPIO_AF1_TIM1;
			__HAL_RCC_TIM1_CLK_ENABLE();
		}
    else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM2)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			//TIMx.Instance = TIM2;
			//GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
			__HAL_RCC_TIM2_CLK_ENABLE();
    }
    else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM3)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			//TIMx.Instance = TIM3;
			//GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
			__HAL_RCC_TIM3_CLK_ENABLE();
    }else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM4)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			//TIMx.Instance = TIM4;
			//GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;
			__HAL_RCC_TIM4_CLK_ENABLE();
    }
		#ifdef TIM5
		else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM5)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			//TIMx.Instance = TIM5;
			//GPIO_InitStructure.Alternate = GPIO_AF2_TIM5;
			__HAL_RCC_TIM5_CLK_ENABLE();
    }
		#endif
		#ifdef TIM8
		else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM8)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM8, ENABLE);
			//TIMx.Instance = TIM8;
			//GPIO_InitStructure.Alternate = GPIO_AF3_TIM8;
			__HAL_RCC_TIM8_CLK_ENABLE();
    }
		#endif
		#ifdef TIM9
		else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM9)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM9, ENABLE);
			//TIMx.Instance = TIM9;
			//GPIO_InitStructure.Alternate = GPIO_AF3_TIM9;
			__HAL_RCC_TIM9_CLK_ENABLE();
    }
		#endif
		#ifdef TIM10
		else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM10)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM10, ENABLE);
			//TIMx.Instance = TIM10;
			//GPIO_InitStructure.Alternate = GPIO_AF3_TIM10;
			__HAL_RCC_TIM10_CLK_ENABLE();
    }
		#endif
//		__HAL_RCC_GPIOD_CLK_ENABLE();
//			GPIO_InitStructure.Pin = gpio_pin;
//		 GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//		 GPIO_InitStructure.Pull =  GPIO_NOPULL;
//		 GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//		 //GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//		HAL_GPIO_Init(gpio_port, &GPIO_InitStructure);////////////////HAL_TIM_MspPostInit(&TIMx);
//    

			TIMx.Instance =g_APinDescription[ulPin].ulTimerPeripheral;
			// Time base configuration
			TIMx.Init.Period = PWM_Manager.arr;
			TIMx.Init.Prescaler = PWM_Manager.presc;
			TIMx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//0
			TIMx.Init.CounterMode = TIM_COUNTERMODE_UP;			
			HAL_TIM_Base_Init(&TIMx);
		
			sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
			HAL_TIM_ConfigClockSource(&TIMx, &sClockSourceConfig);

			HAL_TIM_PWM_Init(&TIMx);
		
			sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
			sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
			HAL_TIMEx_MasterConfigSynchronization(&TIMx, &sMasterConfig);
		
			pinEnabled[ulPin] = 1;
		  }
	 
			TIMx.Instance =g_APinDescription[ulPin].ulTimerPeripheral;			
			// PWM1 Mode configuration
			TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
			TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;			
			TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
			TIM_OCInitStructure.Pulse = 100;
		
			if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_1)
			{
				// PWM1 Mode configuration: Channel1
				HAL_TIM_PWM_ConfigChannel(&TIMx, &TIM_OCInitStructure, TIM_CHANNEL_1);		
				HAL_TIM_PWM_Start(&TIMx, TIM_CHANNEL_1);
			}
			else if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_2)
			{
				// PWM1 Mode configuration: Channel2
				HAL_TIM_PWM_ConfigChannel(&TIMx, &TIM_OCInitStructure, TIM_CHANNEL_2);	
				HAL_TIM_PWM_Start(&TIMx, TIM_CHANNEL_2);
			}
			else if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_3)
			{
				// PWM1 Mode configuration: Channel3
				HAL_TIM_PWM_ConfigChannel(&TIMx, &TIM_OCInitStructure, TIM_CHANNEL_3);				
				HAL_TIM_PWM_Start(&TIMx, TIM_CHANNEL_3);
			}
			else if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_4)
			{
				// PWM1 Mode configuration: Channel4
				HAL_TIM_PWM_ConfigChannel(&TIMx, &TIM_OCInitStructure, TIM_CHANNEL_4);							
				HAL_TIM_PWM_Start(&TIMx, TIM_CHANNEL_4);
			}
		
//			TIM_ARRPreloadConfig(g_APinDescription[ulPin].ulTimerPeripheral, ENABLE);
//		
//			// TIM enable counter
//			TIM_Cmd(g_APinDescription[ulPin].ulTimerPeripheral, ENABLE);
//		
//		  //for TIM1 and TIM8
//		  TIM_CtrlPWMOutputs(g_APinDescription[ulPin].ulTimerPeripheral, ENABLE);			

			
	
	
}

// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void pwmWrite(uint32_t ulPin, uint32_t ulValue)
{
	
		  //PWM Frequency : 1000 Hz,Timer counter clk:1MHz
	
	
	uint16_t TIM_CCR = ulValue;
	   
	   if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_1)
		   {
			   // PWM1 Mode configuration: Channel1
			  g_APinDescription[ulPin].ulTimerPeripheral->CCR1 = TIM_CCR;
		   }
		   else if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_2)
		   {
			   // PWM1 Mode configuration: Channel2
			   g_APinDescription[ulPin].ulTimerPeripheral->CCR2 = TIM_CCR;
		   }
		   else if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_3)
		   {
			   // PWM1 Mode configuration: Channel3
			   g_APinDescription[ulPin].ulTimerPeripheral->CCR3 = TIM_CCR;
		   }
		   else if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_4)
		   {
			   // PWM1 Mode configuration: Channel4
			  g_APinDescription[ulPin].ulTimerPeripheral->CCR4 = TIM_CCR;
		   }
	

	return;
}
//////////////////////////////////
int _readResolution = 10;
int _writeResolution = 8;

 uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	if (from == to)
		return value;
	if (from > to)
		return value >> (from-to);
	else
		return value << (to-from);
}
 
void analogWrite(uint32_t ulPin, uint32_t ulValue) {
	
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_HandleTypeDef  TIMx;
	TIM_OC_InitTypeDef   TIM_OCInitStructure;
	TIM_MasterConfigTypeDef sMasterConfig;
	
	
	uint16_t TIM_Prescaler=0,TIM_ARR=0,Duty_Cycle=0,TIM_CCR=0;

	if ( g_APinDescription[ulPin].ulTimerPeripheral == NULL)
	{
    // Defaults to digital write
    pinMode(ulPin, OUTPUT);
    ulValue = mapResolution(ulValue, _writeResolution, 8);
    if (ulValue < 128)
      digitalWrite(ulPin, LOW);
    else
      digitalWrite(ulPin, HIGH);
	

    return;
  }


  ulValue = mapResolution(ulValue, _writeResolution, PWM_RESOLUTION);       //??PWM ???? ???????ulValue.

	//PWM Frequency : 1000 Hz,Timer counter clk:1MHz 
	 TIM_Prescaler = (uint16_t)(F_CPU/1000000 - 1);//
	 TIM_ARR = (uint16_t)(1000000 / PWM_FREQUENCY) - 1;

	 Duty_Cycle = (uint16_t)((ulValue * 100) / 255);
	// TIM Channel Duty Cycle(%) = (TIM_CCR / TIM_ARR + 1) * 100
	 TIM_CCR = (uint16_t)((Duty_Cycle * (TIM_ARR + 1)) / 100);
	////
  	
	

  if (!pinEnabled[ulPin]) {
		
//	GPIO_InitTypeDef GPIO_InitStructure;
//		GPIO_TypeDef *gpio_port = g_APinDescription[ulPin].pPort;
//		uint16_t gpio_pin = g_APinDescription[ulPin].ulPin;
	
		__HAL_RCC_AFIO_CLK_ENABLE();
		pinMode(ulPin, AF_PP);//
			
		// TIM clock enable
    if(g_APinDescription[ulPin].ulTimerPeripheral == TIM1)
		{
			//RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
			//TIMx.Instance = TIM1;
			//GPIO_InitStructure.Alternate = GPIO_AF1_TIM1;
			__HAL_RCC_TIM1_CLK_ENABLE();
		}
    else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM2)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			//TIMx.Instance = TIM2;
			//GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
			__HAL_RCC_TIM2_CLK_ENABLE();
    }
    else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM3)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
			//TIMx.Instance = TIM3;
			//GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;
			__HAL_RCC_TIM3_CLK_ENABLE();
    }else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM4)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			//TIMx.Instance = TIM4;
			//GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;
			__HAL_RCC_TIM4_CLK_ENABLE();
    }
		#ifdef TIM5
		else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM5)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
			//TIMx.Instance = TIM5;
			//GPIO_InitStructure.Alternate = GPIO_AF2_TIM5;
			__HAL_RCC_TIM5_CLK_ENABLE();
    }
		#endif
		#ifdef TIM8
		else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM8)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM8, ENABLE);
			//TIMx.Instance = TIM8;
			//GPIO_InitStructure.Alternate = GPIO_AF3_TIM8;
			__HAL_RCC_TIM8_CLK_ENABLE();
    }
		#endif
		#ifdef TIM9
		else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM9)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM9, ENABLE);
			//TIMx.Instance = TIM9;
			//GPIO_InitStructure.Alternate = GPIO_AF3_TIM9;
			__HAL_RCC_TIM9_CLK_ENABLE();
    }
		#endif
		#ifdef TIM10
		else if(g_APinDescription[ulPin].ulTimerPeripheral == TIM10)
    {
      //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM10, ENABLE);
			//TIMx.Instance = TIM10;
			//GPIO_InitStructure.Alternate = GPIO_AF3_TIM10;
			__HAL_RCC_TIM10_CLK_ENABLE();
    }
		#endif
//		__HAL_RCC_GPIOD_CLK_ENABLE();
//			GPIO_InitStructure.Pin = gpio_pin;
//		 GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//		 GPIO_InitStructure.Pull =  GPIO_NOPULL;
//		 GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//		 //GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//		HAL_GPIO_Init(gpio_port, &GPIO_InitStructure);//////////////
    // Setup PWM for this pin	
		
		TIMx.Instance =g_APinDescription[ulPin].ulTimerPeripheral;
			// Time base configuration
			TIMx.Init.Period = TIM_ARR;
			TIMx.Init.Prescaler = TIM_Prescaler;
			TIMx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//0
			TIMx.Init.CounterMode = TIM_COUNTERMODE_UP;			
			HAL_TIM_Base_Init(&TIMx);
		
			sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
			HAL_TIM_ConfigClockSource(&TIMx, &sClockSourceConfig);
		
			HAL_TIM_PWM_Init(&TIMx);
		
			sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
			sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
			HAL_TIMEx_MasterConfigSynchronization(&TIMx, &sMasterConfig);			
		
			
    pinEnabled[ulPin] = 1;
  }

	// PWM1 Mode configuration
			TIMx.Instance = g_APinDescription[ulPin].ulTimerPeripheral;
	
			// PWM1 Mode configuration
			TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
			TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;			
			TIM_OCInitStructure.OCPolarity = TIM_OCPOLARITY_HIGH;
			TIM_OCInitStructure.Pulse = TIM_CCR;
		
			if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_1)
			{
				// PWM1 Mode configuration: Channel1
				HAL_TIM_PWM_ConfigChannel(&TIMx, &TIM_OCInitStructure, TIM_CHANNEL_1);		
				HAL_TIM_PWM_Start(&TIMx, TIM_CHANNEL_1);
			}
			else if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_2)
			{
				// PWM1 Mode configuration: Channel2
				HAL_TIM_PWM_ConfigChannel(&TIMx, &TIM_OCInitStructure, TIM_CHANNEL_2);	
				HAL_TIM_PWM_Start(&TIMx, TIM_CHANNEL_2);
			}
			else if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_3)
			{
				// PWM1 Mode configuration: Channel3
				HAL_TIM_PWM_ConfigChannel(&TIMx, &TIM_OCInitStructure, TIM_CHANNEL_3);				
				HAL_TIM_PWM_Start(&TIMx, TIM_CHANNEL_3);
			}
			else if(g_APinDescription[ulPin].ulTimerChannel == TIM_CHANNEL_4)
			{
				// PWM1 Mode configuration: Channel4
				HAL_TIM_PWM_ConfigChannel(&TIMx, &TIM_OCInitStructure, TIM_CHANNEL_4);							
				HAL_TIM_PWM_Start(&TIMx, TIM_CHANNEL_4);

			}
}

/////////////////////////////////////

//void adcMode(uint32_t ulPin, uint8_t Mode)
//{

////  	  ADC_InitTypeDef ADC_InitStructure;
////	  GPIO_InitTypeDef GPIO_InitStructure;
//	

//}

//uint32_t adcRead(uint32_t ulPin)
//{
//	uint32_t ulValue = 0;
//	uint32_t ulChannel;


//	ulChannel = g_APinDescription[ulPin].ulADCChannelNumber ;	

//	  
//	if ( ulChannel == NULL )
//	{
//		return -1;
//	}
////	static int enabled = 0;
////	if (!enabled) {
////		adcMode(ulPin, ADC_12_BIT);
////		enabled = 1;
////	}

//	/* ADC1 regular channel14 configuration */ 
//	  ADC_RegularChannelConfig(ADC1, g_APinDescription[ulPin].ulADCChannelNumber, 1, ADC_SampleTime_55Cycles5);
//		
//	
//	//Start ADC1 Software Conversion
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

//	// Wait until conversion completion
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

//	// Read the value
//	ulValue = ADC_GetConversionValue(ADC1);
//	ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);

//	return ulValue;



//}

uint32_t analogRead(uint32_t ulPin)
{
	uint32_t ulValue = 0;
	uint32_t ulChannel;
		ADC_ChannelConfTypeDef sConfig;
		ADC_HandleTypeDef hadc1;
		uint8_t Mode=ADC_8_BIT;
	

	static int enabled = 0;
	
	ulChannel = g_APinDescription[ulPin].ulADCChannelNumber ;		  
//	if ( ulChannel == NULL )
//	{
//		return -1;
//	}
	
	if (!enabled) {
		//adcMode(ulPin, ADC_12_BIT);
		
		//	  /*config GPIO, ADC RCC*/
		pinMode(ulPin, AN_INPUT);		
			 /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();			
			
			hadc1.Instance = ADC1;
			//hadc1.Init.Resolution = ADC_RESOLUTION_12B;
			hadc1.Init.ScanConvMode = DISABLE;
			hadc1.Init.ContinuousConvMode = DISABLE;
			hadc1.Init.DiscontinuousConvMode = DISABLE;
			//hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
			hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
			hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
			//hadc1.Init.DMAContinuousRequests = DISABLE;
			hadc1.Init.NbrOfDiscConversion = 0;
			hadc1.Init.NbrOfConversion = 1;
			//hadc1.Init.EOCSelection = DISABLE;
			//hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
			HAL_ADC_Init(&hadc1);
	  

      switch(Mode)	  
      {
			case ADC_8_BIT:  _readResolution = 8; break;
			case ADC_10_BIT:  _readResolution = 10; break;
			case ADC_12_BIT: _readResolution = 12; break;
			default: 
				 _readResolution = 10; break;
			}
		
		enabled = 1;delay(200);
	}
	
	/**Configure Regular Channel 
				*/
			sConfig.Channel = (uint8_t)g_APinDescription[ulPin].ulADCChannelNumber;
			sConfig.Rank = 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
			
			hadc1.Instance = ADC1;
			//HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			/* Return zero */
			if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
				return 0;
			}	
			
			
	/* Start conversion */  
	if (HAL_ADC_Start(&hadc1) != HAL_OK) {
		return 0;
	}
	/* Poll for end */
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
		/* Get the converted value of regular channel */		
		// Read the value
	//if (HAL_ADC_GetState(&hadc1) == HAL_ADC_STATE_EOC_REG){
		ulValue = HAL_ADC_GetValue(&hadc1);
		ulValue = mapResolution(ulValue, ADC_RESOLUTION, _readResolution);

	return ulValue;//}
	}


	return 0;
}
///////////////////////////////////////
//extern void Error_Handler(void);
//UART_HandleTypeDef huartdma;
//DMA_HandleTypeDef hdma_usart2_rx;
//DMA_HandleTypeDef hdma_usart2_tx;
//UART_HandleTypeDef uartdma;
//DMA_HandleTypeDef hdma_usart3_rx;
//DMA_HandleTypeDef hdma_usart3_tx;


///* USART2 init function */
//extern "C" void DMA_USARTx_Init(USART_TypeDef* USARTx)
//{

//	
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//  
//    /**USART2 GPIO Configuration    
//    PA2     ------> USART2_TX
//    PA3     ------> USART2_RX 
//    */    
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_PULLUP;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	
//		if(USARTx == USART2)//Serial2 usart2
//			{ 
//				pinMode(TX2, AF_PP);//__HAL_RCC_GPIOA_CLK_ENABLE();
//				pinMode(RX2, AF_PP);//__HAL_RCC_GPIOA_CLK_ENABLE();
//				
//				__HAL_RCC_USART2_CLK_ENABLE();
//				
//				GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
//				
//				GPIO_InitStruct.Pin = g_APinDescription[TX2].ulPin;		
//				HAL_GPIO_Init(g_APinDescription[TX2].pPort, &GPIO_InitStruct);
//				
//				GPIO_InitStruct.Pin =  g_APinDescription[RX2].ulPin;
//				HAL_GPIO_Init(g_APinDescription[RX2].pPort, &GPIO_InitStruct);
//				
//				
//					/* DMA controller clock enable */

//					__HAL_RCC_DMA1_CLK_ENABLE();
//				
//					/* Peripheral DMA init*/
//					huartdma.Instance = USARTx;
//					huartdma.Init.BaudRate = 115200;
//					huartdma.Init.WordLength = UART_WORDLENGTH_8B;
//					huartdma.Init.StopBits = UART_STOPBITS_1;
//					huartdma.Init.Parity = UART_PARITY_NONE;
//					huartdma.Init.Mode = UART_MODE_TX_RX;
//					huartdma.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//					huartdma.Init.OverSampling = UART_OVERSAMPLING_16;
//					if (HAL_UART_Init(&huartdma) != HAL_OK)
//					{
//						Error_Handler();
//					}
//					/* DMA interrupt init */
//					/* DMA1_Stream5_IRQn interrupt configuration */
//					HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
//					HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
//					/* DMA1_Stream6_IRQn interrupt configuration */
//					HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
//					HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
//					
//							
//						hdma_usart2_rx.Instance = DMA1_Stream5;
//						hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
//						hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//						hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//						hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
//						hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//						hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//						hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
//						hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
//						hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//						if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
//						{
//							Error_Handler();
//						}

//						__HAL_LINKDMA(&huartdma,hdmarx,hdma_usart2_rx);

//						hdma_usart2_tx.Instance = DMA1_Stream6;
//						hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
//						hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//						hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//						hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
//						hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//						hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//						hdma_usart2_tx.Init.Mode = DMA_CIRCULAR;
//						hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
//						hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//						if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
//						{
//							Error_Handler();
//						}

//						__HAL_LINKDMA(&huartdma,hdmatx,hdma_usart2_tx);
//				
//				
//			} else if(USARTx == USART3)//Serial2 usart2
//				{ 
//					pinMode(TX3, AF_PP);//__HAL_RCC_GPIOA_CLK_ENABLE();
//					pinMode(RX3, AF_PP);//__HAL_RCC_GPIOA_CLK_ENABLE();
//					
//					__HAL_RCC_USART3_CLK_ENABLE();
//					
//					GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//					
//					GPIO_InitStruct.Pin = g_APinDescription[TX3].ulPin;		
//					HAL_GPIO_Init(g_APinDescription[TX3].pPort, &GPIO_InitStruct);
//					
//					GPIO_InitStruct.Pin =  g_APinDescription[RX3].ulPin;
//					HAL_GPIO_Init(g_APinDescription[RX3].pPort, &GPIO_InitStruct);
//					/* DMA controller clock enable */
//					__HAL_RCC_DMA1_CLK_ENABLE();
//					/* Peripheral DMA init*/
//					uartdma.Instance = USARTx;
//					uartdma.Init.BaudRate = 115200;
//					uartdma.Init.WordLength = UART_WORDLENGTH_8B;
//					uartdma.Init.StopBits = UART_STOPBITS_1;
//					uartdma.Init.Parity = UART_PARITY_NONE;
//					uartdma.Init.Mode = UART_MODE_TX_RX;
//					uartdma.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//					uartdma.Init.OverSampling = UART_OVERSAMPLING_16;
//					if (HAL_UART_Init(&uartdma) != HAL_OK)
//					{
//						Error_Handler();
//					}
//					/* DMA interrupt init */					
//					/* DMA1_Stream1_IRQn interrupt configuration */
//					HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
//					HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
//					/* DMA1_Stream3_IRQn interrupt configuration */
//					HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
//					HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
//					
//						hdma_usart3_rx.Instance = DMA1_Stream1;
//						hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
//						hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
//						hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
//						hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
//						hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//						hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//						hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
//						hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
//						hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//						if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
//						{
//							Error_Handler();
//						}

//						__HAL_LINKDMA(&uartdma,hdmarx,hdma_usart3_rx);

//						hdma_usart3_tx.Instance = DMA1_Stream3;
//						hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
//						hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
//						hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
//						hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
//						hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//						hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//						hdma_usart3_tx.Init.Mode = DMA_CIRCULAR;
//						hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
//						hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//						if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
//						{
//							Error_Handler();
//						}

//						__HAL_LINKDMA(&uartdma,hdmatx,hdma_usart3_tx);
//					
//					
//				}

//    
//  


//}

///**
//* @brief This function handles DMA1 uart3 global interrupt.
//*/
//extern "C" void DMA1_Stream1_IRQHandler(void)
//{

//  HAL_DMA_IRQHandler(&hdma_usart3_rx);
//	
//}


//extern "C" void DMA1_Stream3_IRQHandler(void)
//{
// 
//  HAL_DMA_IRQHandler(&hdma_usart3_tx);
//	
//}
///**
//* @brief This function handles DMA1 uart2 global interrupt.
//*/
//extern "C" void DMA1_Stream5_IRQHandler(void)
//{

//  HAL_DMA_IRQHandler(&hdma_usart2_rx);
//	
//}

//extern "C" void DMA1_Stream6_IRQHandler(void)
//{
// 
//  HAL_DMA_IRQHandler(&hdma_usart2_tx);
//	
//}
//extern "C" void DMA2_Stream7_IRQHandler() {
//	uint32_t isr = DMA2->HISR;
//	if (isr & DMA_HISR_TCIF7) {
//	  DMA2->HIFCR = DMA_HIFCR_CTCIF7;
//	
//    }
//}


// ----------------------------------------------------------------------------
/*
 * USART objects
 */
#include "USARTClass.h"


RingBuffer rx_buffer1;
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;
RingBuffer rx_buffer4;
RingBuffer rx_buffer5;
RingBuffer rx_buffer6;



USARTClass Serial(USART1, USART1_IRQn, id_serial1, &rx_buffer1);


USARTClass Serial1(USART1, USART1_IRQn, id_serial1, &rx_buffer1);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }


USARTClass Serial2(USART2, USART2_IRQn, id_serial2, &rx_buffer2);
void serialEvent2() __attribute__((weak));
void serialEvent2() { }


USARTClass Serial3(USART3, USART3_IRQn, id_serial3, &rx_buffer3);
void serialEvent3() __attribute__((weak));
void serialEvent3() { }


#ifdef UART4
USARTClass Serial4(UART4, UART4_IRQn, id_serial4, &rx_buffer4);
void serialEvent4() __attribute__((weak));
void serialEvent4() { }
#endif

#ifdef UART5
USARTClass Serial5(UART5, UART5_IRQn, id_serial5, &rx_buffer5);
void serialEvent5() __attribute__((weak));
void serialEvent5() { }
#endif

#ifdef USART6
USARTClass Serial6(USART6, USART6_IRQn, id_serial6, &rx_buffer6);
void serialEvent6() __attribute__((weak));
void serialEvent6() { }
#endif

#include "HardwareSerial.h"
// IT handlers
extern "C" void USART1_IRQHandler(void) 
{

		Serial1.IrqHandler();//USART1 must be Serial1,for usart flash programming.
		Serial.IrqHandler();
	
}

extern "C" void USART2_IRQHandler(void) 
{
  Serial2.IrqHandler();

}

extern "C" void USART3_IRQHandler(void) 
{
  Serial3.IrqHandler();
}

#ifdef UART4
extern "C" void UART4_IRQHandler(void) 
{

  Serial4.IrqHandler();
  
}
#endif
#ifdef UART5
extern "C" void UART5_IRQHandler(void)
{
	
  Serial5.IrqHandler();


}
#endif
#ifdef USART6
extern "C" void USART6_IRQHandler(void)
{
	
  Serial6.IrqHandler();


}
#endif

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
 
  if (Serial1.available()) serialEvent1();
  if (Serial2.available()) serialEvent2();
  if (Serial3.available()) serialEvent3();
	#ifdef UART4
  if (Serial4.available()) serialEvent4();
	#endif
	#ifdef UART5
  if (Serial5.available()) serialEvent5();
	#endif
	#ifdef USART6
	if (Serial6.available()) serialEvent6();
	#endif

}
////////////////////////////////////////////////////////////////////////////////////
#include "SPI.h"/////////////////////////////////////////////////////////////////////


static SPI_HandleTypeDef hspi = {0};

#define MOSI1  PIN_SPI_MOSI1
#define MISO1  PIN_SPI_MISO1
#define SCK1   PIN_SPI_SCK1

#define MOSI2  PIN_SPI_MOSI2
#define MISO2  PIN_SPI_MISO2
#define SCK2   PIN_SPI_SCK2

#define MOSI3  PIN_SPI_MOSI3
#define MISO3  PIN_SPI_MISO3
#define SCK3   PIN_SPI_SCK3


SPIClass::SPIClass(SPI_TypeDef *_spi, Funct _initCb) :
spi(_spi),  initCb(_initCb), initialized(false),bitOrder(SPI_FIRSTBIT_MSB),
  mode(SPI_MODE0),divider(SPI_CLOCK_DIV32)
{
	
}

void SPIClass::begin() {
	
	init();

	setdataSize(SPI_DATASIZE_8BIT);//dataSize = SPI_DataSize_8b;
	setClockDivider(divider);
	setDataMode( mode);
	setBitOrder( bitOrder);
	//printf("%d--%d--%d",divider,mode,bitOrder);
	
	
}

#if 0
void SPIClass::begin(uint8_t _pin) {
	init();

	uint32_t spiPin = BOARD_PIN_TO_SPI_PIN(_pin);
	PIO_Configure(
		g_APinDescription[spiPin].pPort,
		g_APinDescription[spiPin].ulPinType,
		g_APinDescription[spiPin].ulPin,
		g_APinDescription[spiPin].ulPinConfiguration);

	// Default speed set to 4Mhz
	setClockDivider(_pin, 21);
	setDataMode(_pin, SPI_MODE0);
	setBitOrder(_pin, MSBFIRST);
}
#endif

void SPIClass::init() {
	
	if (initialized)
		return;
	initCb();
//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
//  SPI_InitStructure.SPI_CRCPolynomial = 7;

//  SPI_Init(spi,&SPI_InitStructure);
//  SPI_Cmd(spi,ENABLE);
	
	
	/*##-1- Configure the SPI peripheral #######################################*/
	/* Set the SPI parameters */
	hspi.Instance               = spi;
//	hspi.Init.BaudRatePrescaler = _BaudRatePrescaler;
	hspi.Init.Direction         = SPI_DIRECTION_2LINES;
	//hspi.Init.CLKPhase          = SPI_PHASE_1EDGE;
	//hspi.Init.CLKPolarity       = SPI_POLARITY_LOW;
	hspi.Init.DataSize          = dataSize;//SPI_DATASIZE_8BIT;
	hspi.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	hspi.Init.TIMode            = SPI_TIMODE_DISABLE;
	hspi.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	hspi.Init.CRCPolynomial     = 7;
//	hspi.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;
	hspi.Init.NSS               = SPI_NSS_SOFT;
//	hspi.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
//	hspi.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
//	hspi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;  /* Recommanded setting to avoid glitches */
	hspi.Init.Mode 			 = SPI_MODE_MASTER;

/* Disable first */
	__HAL_SPI_DISABLE(&hspi);
	
	if (HAL_SPI_Init(&hspi) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	
	/* Enable SPI */
	__HAL_SPI_ENABLE(&hspi);
	initialized = true;
}

const uint32_t baud_rates[8]  = {
    SPI_CLOCK_DIV2,
    SPI_CLOCK_DIV4,
    SPI_CLOCK_DIV8,
    SPI_CLOCK_DIV16,
    SPI_CLOCK_DIV32,
    SPI_CLOCK_DIV64,
    SPI_CLOCK_DIV128,
    SPI_CLOCK_DIV256,
};
///////////////////////////////////chua test////////////////////////////////////
/*
* Note: This assumes you're on a LeafLabs-style board
* (CYCLES_PER_MICROSECOND == 84, APB2 at 84MHz, APB1 at 42MHz).
	determine_baud_rate(spi, 2000000);
*/
static uint16_t determine_baud_rate(SPI_TypeDef *_spi, uint32_t freq) {
    uint32_t clock;
    
    if (_spi==SPI1) clock = HAL_RCC_GetPCLK2Freq();//STM32_PCLK2;  // 84 Mhz
    else if (_spi==SPI2) clock = (HAL_RCC_GetPCLK1Freq());//STM32_PCLK1;  // 42 Mhz
		#ifdef SPI3
    else if (_spi==SPI3)clock = (HAL_RCC_GetPCLK1Freq()); 
		#endif
		//There is no SPI on this bus, but it removes warning 
    
    clock /= 2;
    uint32_t i = 0;
    while (i < 7 && freq < clock) {
        clock /= 2;
        i++;
    }
    return baud_rates[i];
}
/////////////////////////////////////////////////////////////
#if 0
void SPIClass::end(uint8_t _pin) {
	uint32_t spiPin = BOARD_PIN_TO_SPI_PIN(_pin);
	// Setting the pin as INPUT will disconnect it from SPI peripheral
	pinMode(spiPin, INPUT);
}
#endif

void SPIClass::end() {
	//SPI_Cmd(spi,DISABLE);
	spi->CR1&=  ~SPI_CR1_SPE;//__HAL_SPI_DISABLE(hspi);
	initialized = false;
}

void SPIClass::setBitOrder( uint16_t _bitOrder) {

  spi->CR1&=  ~SPI_CR1_SPE;//SPI_Cmd(spi,DISABLE);
  hspi.Init.FirstBit          = _bitOrder;//SPI_InitStructure.SPI_FirstBit = _bitOrder;
  bitOrder = _bitOrder;

//  SPI_Init(spi,&SPI_InitStructure);
//  SPI_Cmd(spi,ENABLE);
	if (HAL_SPI_Init(&hspi) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	/* Enable SPI */
	__HAL_SPI_ENABLE(&hspi);
}
void SPIClass::setdataSize( uint16_t _dataSize) {

  spi->CR1&=  ~SPI_CR1_SPE;//SPI_Cmd(spi,DISABLE);__HAL_SPI_DISABLE(&hspi);
  hspi.Init.DataSize = _dataSize;//SPI_InitStructure.SPI_DataSize = _dataSize;  
	dataSize = _dataSize;

//  SPI_Init(spi,&SPI_InitStructure);
//  SPI_Cmd(spi,ENABLE);
	if (HAL_SPI_Init(&hspi) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	/* Enable SPI */
	__HAL_SPI_ENABLE(&hspi);
}
void SPIClass::setDataMode( uint8_t _mode) {

  mode = _mode;
  spi->CR1&=  ~SPI_CR1_SPE;//SPI_Cmd(spi,DISABLE);
  if(_mode == SPI_MODE0)
  {
		hspi.Init.CLKPhase          = SPI_PHASE_1EDGE ;
		hspi.Init.CLKPolarity       = SPI_POLARITY_LOW;
//    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
//    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  } else if(_mode == SPI_MODE1)
  {
    hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi.Init.CLKPhase  = SPI_PHASE_2EDGE ;
  } else if(_mode == SPI_MODE2)
  {
    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase  = SPI_PHASE_1EDGE ;
  } else if(_mode == SPI_MODE3)
  {
    hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi.Init.CLKPhase  = SPI_PHASE_2EDGE ;
  }

//  SPI_Init(spi,&SPI_InitStructure);
//  SPI_Cmd(spi,ENABLE);
	if (HAL_SPI_Init(&hspi) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	/* Enable SPI */
	__HAL_SPI_ENABLE(&hspi);
}

void SPIClass::setClockDivider( uint8_t _divider) {

  spi->CR1&=  ~SPI_CR1_SPE;//SPI_Cmd(spi,DISABLE);
  hspi.Init.BaudRatePrescaler = _divider;//SPI_InitStructure.SPI_BaudRatePrescaler = _divider;
  divider = _divider;
	//printf("/n%d--%d--%d\n",divider,mode,bitOrder);

//  SPI_Init(spi,&SPI_InitStructure);
//  SPI_Cmd(spi,ENABLE);
	if (HAL_SPI_Init(&hspi) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	/* Enable SPI */
	__HAL_SPI_ENABLE(&hspi);
}

void SPIClass::setClock( uint32_t freq ) {

  spi->CR1&=  ~SPI_CR1_SPE;//SPI_Cmd(spi,DISABLE);__HAL_SPI_DISABLE(&hspi);
  hspi.Init.BaudRatePrescaler = determine_baud_rate(spi, freq);
  divider = determine_baud_rate(spi, freq);
	//printf("/n%d--%d--%d\n",divider,mode,bitOrder);

//  SPI_Init(spi,&SPI_InitStructure);
//  SPI_Cmd(spi,ENABLE);
	if (HAL_SPI_Init(&hspi) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	/* Enable SPI */
	__HAL_SPI_ENABLE(&hspi);
}


/**
 * @brief  Check SPI busy status
 */
#define SPI_IS_BUSY(SPIx)                   (((SPIx)->SR & (SPI_SR_TXE | SPI_SR_RXNE)) == 0)

/**
 * @brief  SPI wait till end
 */
#define SPI_WAIT_TX(SPIx)                   while ((SPIx->SR & SPI_FLAG_TXE) == 0 || (SPIx->SR & SPI_FLAG_BSY))
#define SPI_WAIT_RX(SPIx)                   while ((SPIx->SR & SPI_FLAG_RXNE) == 0 || (SPIx->SR & SPI_FLAG_BSY))

/**
 * @brief  Checks if SPI is enabled
 */
#define SPI_CHECK_ENABLED(SPIx)             if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return;}

/**
 * @brief  Checks if SPI is enabled and returns value from function if not 
 */
#define SPI_CHECK_ENABLED_RESP(SPIx, val)   if (!((SPIx)->CR1 & SPI_CR1_SPE)) {return (val);}

uint8_t SPIClass::transfer( uint8_t _data)
 {
	 
//uint8_t DATA=0;
	 
//	if(HAL_SPI_TransmitReceive(&hspi, (uint8_t*)&_data, (uint8_t *)&DATA, 1, 1000000) != HAL_OK)	
//	{
//		Error_Handler();
//	}
	/* Check if SPI is enabled */
	//SPI_CHECK_ENABLED_RESP(spi, 0);
	
//	if((spi->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
//    {
//      /* Enable SPI peripheral */
//      __HAL_SPI_ENABLE(&hspi);
//    }
	
	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	SPI_WAIT_TX(spi);
	
	/* Fill output buffer with data */
	spi->DR = _data;
	
	/* Wait for transmission to complete */
	SPI_WAIT_RX(spi);
	
	/* Return data from buffer */
	return spi->DR;//SPI_I2S_ReceiveData(spi);//->SPI_RDR;
}
 
uint16_t SPIClass::transfer16( uint16_t _data)
 {
	 
//uint8_t DATA=0;
	 
//	if(HAL_SPI_TransmitReceive(&hspi, (uint8_t*)&_data, (uint8_t *)&DATA, 1, 1000000) != HAL_OK)	
//	{
//		Error_Handler();
//	}
	/* Check if SPI is enabled */
	SPI_CHECK_ENABLED_RESP(spi, 0);
	
	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	SPI_WAIT_TX(spi);
	
	/* Fill output buffer with data */
	spi->DR = _data;
	
	/* Wait for transmission to complete */
	SPI_WAIT_RX(spi);
	
	/* Return data from buffer */
	return spi->DR;//SPI_I2S_ReceiveData(spi);//->SPI_RDR;
}
 
void SPIClass::send16(uint16_t data)
{

		transfer(data>>8);transfer(data);
		 
		//	while((spi->SR&SPI_FLAG_TXE)==RESET);
		//	spi->DR=data>>8;
		//	while((spi->SR&SPI_FLAG_TXE)==RESET);		
		//	while((spi->SR&SPI_FLAG_BSY)!=RESET);
		//	spi->DR=data;		
}

void SPIClass::write(uint8_t data)
{
 
	//transfer(data);
	
	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	SPI_WAIT_TX(spi);
	
	/* Fill output buffer with data */
	spi->DR = data;
	
	/* Wait for transmission to complete */
	SPI_WAIT_RX(spi);
	
//	while((spi->SR&SPI_FLAG_TXE)==RESET);		
//	while((spi->SR&SPI_FLAG_BSY)!=RESET);
//	spi->DR=data;	 	 
//	while((spi->SR&SPI_I2S_FLAG_RXNE)==RESET);
//	//return spi->DR;          	     
}

void SPIClass::write16(const uint16_t data)
{
	//transfer(data);
	
	/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
	SPI_WAIT_TX(spi);
	
	/* Fill output buffer with data */
	spi->DR = data;
	
	/* Wait for transmission to complete */
	SPI_WAIT_RX(spi);
	
//	while((spi->SR&SPI_FLAG_TXE)==RESET);		
//	while((spi->SR&SPI_FLAG_BSY)!=RESET);
//	spi->DR=data;	 	 
//	while((spi->SR&SPI_I2S_FLAG_RXNE)==RESET);
//	//return spi->DR;  
}
void SPIClass::write(const uint16_t data, uint32_t n)
{
    
    while ( (n--)>0 ) {
			
			/* Wait for previous transmissions to complete if DMA TX enabled for SPI */
			SPI_WAIT_TX(spi);
			
			/* Fill output buffer with data */
			spi->DR = data;
			
			/* Wait for transmission to complete */
			SPI_WAIT_RX(spi);
        //spi->DR = data; // write the data to be transmitted into the SPI_DR register (this clears the TXE flag)
        //while ( (spi->SR & SPI_SR_TXE)==0 ) ; // wait till Tx empty
//			while((spi->SR&SPI_I2S_FLAG_TXE)==RESET);
//			spi->DR=data>>8;
//			while((spi->SR&SPI_I2S_FLAG_TXE)==RESET);		
//			//while((spi->SR&SPI_I2S_FLAG_BSY)!=RESET);
//			spi->DR=data;
    }
    //while ( (spi->SR & SPI_SR_BSY) != 0); // wait until BSY=0 before returning 
}

//-----------------------------------------------------------------------------
	//DMA_CH3;//SPI1TX
  //DMA_CH2;//RX
	
	//DMA_CH5;//SPI2TX
  //DMA_CH4;//RX

	//DMA_CH2;//SPI3TX
  //DMA_CH1;//RX
void SPIClass::DMA_Init(const DMA_TypeDef  *dev, DMA_Channel_TypeDef *channel)
{
	dma_init(dev,channel);
	
		if (spi==SPI1) 			channelTx1 = channel;//DMA1_Channel3; // 72 Mhz
    else if (spi==SPI2) channelTx2 = channel;//DMA1_Channel5;  // 36 Mhz
		#ifdef SPI3
    else if (spi==SPI3) channelTx3 = channel;//DMA1_Channel2; // 36 Mhz
		#endif
}
void SPIClass::dmaSend(const void * txBuf, uint16_t length, uint16_t flags)
{
    //flags=((DMA_SxCR_CIRC|DMA_IT_HT) | DMA_SxCR_MINC);	
		DMA_Channel_TypeDef *channel;
	
		if (spi==SPI1) 			channel = channelTx1;//DMA1_Channel3; // 72 Mhz
    else if (spi==SPI2) channel = channelTx2;//DMA1_Channel5;  // 36 Mhz
		#ifdef SPI3
    else if (spi==SPI3) channel = channelTx3;//DMA1_Channel2; // 36 Mhz
		#endif
	
		spi->CR2 |= SPI_CR2_TXDMAEN;
		DMA1->IFCR |= DMA_IFCR_CTCIF1|DMA_IFCR_CTCIF2|DMA_IFCR_CTCIF3|
									DMA_IFCR_CTCIF4|DMA_IFCR_CTCIF5|DMA_IFCR_CTCIF6|DMA_IFCR_CTCIF7;
		dma_xfer_size dma_bit_size = (dataSize==SPI_DATASIZE_16BIT) ? DMA_SIZE_16BITS : DMA_SIZE_8BITS;
    dma_setup_transfer(DMA1, 
											 //DMA2_Stream3,
                       channel,
											 dma_bit_size,
                       &spi->DR,  // peripheral address
                       txBuf,       // memory bank 0 address
	(((flags?(DMA_PRIORITY_VERY_HIGH|DMA_CIRCULAR):0)|DMA_MINC_ENABLE) | ( DMA_MEMORY_TO_PERIPH|DMA_IT_TC)));//DMA_IT_HT|DMA_MEMORY_TO_PERIPH=DMA_SxCR_DIR|DMA_SxCR_MINC |
	
		dma_set_num_transfers(DMA1, channel, length);
		dma_enable(DMA1, channel);
	
	SPI_WAIT_TX(spi);
	SPI_WAIT_RX(spi);
	
	
	
		//	while((spi->SR&SPI_FLAG_TXE)==RESET);
		//	while((spi->SR&SPI_FLAG_RXNE)==RESET);		
		//	while((spi->SR&SPI_FLAG_BSY)!=RESET);	
    
}
//////////////////////////////////////////////////////////

void SPIClass::attachInterrupt(void) {
	// Should be enableInterrupt()
}

void SPIClass::detachInterrupt(void) {
	// Should be disableInterrupt()
	
}



static void SPI_1_Init(void) {

//	GPIO_InitTypeDef GPIO_InitStructure;
		
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI1,ENABLE);
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO,ENABLE);		// Enable clock GPIO
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_SPI1_FORCE_RESET();
	__HAL_RCC_SPI1_RELEASE_RESET();
	__HAL_RCC_AFIO_CLK_ENABLE();
  pinMode(MISO1,AF_PP);
  pinMode(MOSI1,AF_PP);
  pinMode(SCK1, AF_PP);
	
//		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//		GPIO_InitStructure.Pull =  GPIO_NOPULL;//
//		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//		//GPIO_InitStructure.Alternate = GPIO_AF5_SPI1;
//		
//		GPIO_InitStructure.Pin = g_APinDescription[MISO1].ulPin;		
//		HAL_GPIO_Init(g_APinDescription[MISO1].pPort, &GPIO_InitStructure);
//		
//		GPIO_InitStructure.Pin =  g_APinDescription[MOSI1].ulPin;
//		HAL_GPIO_Init(g_APinDescription[MOSI1].pPort, &GPIO_InitStructure);
//	
//		GPIO_InitStructure.Pin =  g_APinDescription[SCK1].ulPin;
//		HAL_GPIO_Init(g_APinDescription[SCK1].pPort, &GPIO_InitStructure);

}


SPIClass SPIx(SPI1,  SPI_1_Init);


static void SPI_2_Init(void) {
//  GPIO_InitTypeDef GPIO_InitStructure;
	
	
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO,ENABLE);		// Enable clock GPIO
	__HAL_RCC_SPI2_CLK_ENABLE();
	__HAL_RCC_SPI2_FORCE_RESET();
	__HAL_RCC_SPI2_RELEASE_RESET();
	__HAL_RCC_AFIO_CLK_ENABLE();
  pinMode(MISO2,AF_PP);//CLK_PORT
  pinMode(MOSI2,AF_PP);
  pinMode(SCK2, AF_PP);
		
//		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//		GPIO_InitStructure.Pull =  GPIO_NOPULL;//
//		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//		//GPIO_InitStructure.Alternate = GPIO_AF5_SPI2;
//		
//		GPIO_InitStructure.Pin = g_APinDescription[MISO2].ulPin;		
//		HAL_GPIO_Init(g_APinDescription[MISO2].pPort, &GPIO_InitStructure);
//		
//		GPIO_InitStructure.Pin =  g_APinDescription[MOSI2].ulPin;
//		HAL_GPIO_Init(g_APinDescription[MOSI2].pPort, &GPIO_InitStructure);
//	
//		GPIO_InitStructure.Pin =  g_APinDescription[SCK2].ulPin;
//		HAL_GPIO_Init(g_APinDescription[SCK2].pPort, &GPIO_InitStructure);
	
}

SPIClass SPI(SPI2,  SPI_2_Init);

#ifdef SPI3
static void SPI_3_Init(void) {

//	GPIO_InitTypeDef GPIO_InitStructure;
	
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO,ENABLE);		// Enable clock GPIO
	__HAL_RCC_SPI3_CLK_ENABLE();
	__HAL_RCC_SPI3_FORCE_RESET();
	__HAL_RCC_SPI3_RELEASE_RESET();
  pinMode(MISO3,AF_PP);
  pinMode(MOSI3,AF_PP);
  pinMode(SCK3, AF_PP);
	
//		GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
//		GPIO_InitStructure.Pull =  GPIO_NOPULL;//
//		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;	
//		//GPIO_InitStructure.Alternate = GPIO_AF6_SPI3;
//		
//		GPIO_InitStructure.Pin = g_APinDescription[MISO3].ulPin;		
//		HAL_GPIO_Init(g_APinDescription[MISO3].pPort, &GPIO_InitStructure);
//		
//		GPIO_InitStructure.Pin =  g_APinDescription[MOSI3].ulPin;
//		HAL_GPIO_Init(g_APinDescription[MOSI3].pPort, &GPIO_InitStructure);
//	
//		GPIO_InitStructure.Pin =  g_APinDescription[SCK3].ulPin;
//		HAL_GPIO_Init(g_APinDescription[SCK3].pPort, &GPIO_InitStructure);
//		
}

SPIClass SPI_3(SPI3,  SPI_3_Init);
#endif

////////////////////////////////////DMA///////////////////////////////////////////
#ifdef HAL_DMA_MODULE_ENABLED
extern "C" {
#ifdef DMA1_Channel1
/**
  * @brief  DMA1 Channel1 IRQHandler
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler()
{
  if (DMA1->ISR & DMA_ISR_TCIF1) {
	  DMA1->IFCR = DMA_IFCR_CTCIF1;	
    }
}
#endif

#ifdef DMA1_Channel2
/**
  * @brief  DMA1 Channel2 IRQHandler
  * @param  None
  * @retval None
  */
void DMA1_Channel2_IRQHandler()
{
  if (DMA1->ISR & DMA_ISR_TCIF2) {
	  DMA1->IFCR = DMA_IFCR_CTCIF2;	
    }
}
#endif

#ifdef DMA1_Channel3
/**
  * @brief  DMA1 Channel3 IRQHandler
  * @param  None
  * @retval None
  */
void DMA1_Channel3_IRQHandler()
{
  if (DMA1->ISR & DMA_ISR_TCIF3) {
	  DMA1->IFCR = DMA_IFCR_CTCIF3;	
    }
}
#endif

#ifdef DMA1_Channel4
/**
  * @brief  DMA1 Channel4 IRQHandler
  * @param  None
  * @retval None
  */
void DMA1_Channel4_IRQHandler()
{
  if (DMA1->ISR & DMA_ISR_TCIF4) {
	  DMA1->IFCR = DMA_IFCR_CTCIF4;	
    }
}
#endif

#ifdef DMA1_Channel5
/**
  * @brief  DMA1 Channel5 IRQHandler
  * @param  None
  * @retval None
  */
void DMA1_Channel5_IRQHandler()
{
  if (DMA1->ISR & DMA_ISR_TCIF5) {
	  DMA1->IFCR = DMA_IFCR_CTCIF5;	
    }
}
#endif

#ifdef DMA1_Channel6/**
  * @brief  DMA1 Channel6 IRQHandler
  * @param  None
  * @retval None
  */
void DMA1_Channel6_IRQHandler()
{
  if (DMA1->ISR & DMA_ISR_TCIF6) {
	  DMA1->IFCR = DMA_IFCR_CTCIF6;	
    }
}
#endif

#ifdef DMA1_Channel7
/**
  * @brief  DMA1 Channel3 IRQHandler
  * @param  None
  * @retval None
  */
void DMA1_Channel7_IRQHandler()
{
  if (DMA1->ISR & DMA_ISR_TCIF7) {
	  DMA1->IFCR = DMA_IFCR_CTCIF7;	
    }
}
#endif

#ifdef DMA2_Channel1
/**
  * @brief  DMA2 Channel1 IRQHandler
  * @param  None
  * @retval None
  */
void DMA2_Channel1_IRQHandler()
{
  if (DMA2->ISR & DMA_ISR_TCIF1) {
	  DMA2->IFCR = DMA_IFCR_CTCIF1;	
    }
}
#endif

#ifdef DMA2_Channel2
/**
  * @brief  DMA2 Channel2 IRQHandler
  * @param  None
  * @retval None
  */
void DMA2_Channel2_IRQHandler()
{
  if (DMA2->ISR & DMA_ISR_TCIF2) {
	  DMA2->IFCR = DMA_IFCR_CTCIF2;	
    }
}
#endif

#ifdef DMA2_Channel3
/**
  * @brief  DMA2 Channel3 IRQHandler
  * @param  None
  * @retval None
  */
void DMA2_Channel3_IRQHandler()
{
  if (DMA2->ISR & DMA_ISR_TCIF3) {
	  DMA2->IFCR = DMA_IFCR_CTCIF3;	
    }
}
#endif

#ifdef DMA2_Channel4
/**
  * @brief  DMA2 Channel4 IRQHandler
  * @param  None
  * @retval None
  */
void DMA2_Channel4_IRQHandler()
{
  if (DMA2->ISR & DMA_ISR_TCIF4) {
	  DMA2->IFCR = DMA_IFCR_CTCIF4;	
    }
}
#endif

#ifdef DMA2_Channel5
/**
  * @brief  DMA2 Channel5 IRQHandler
  * @param  None
  * @retval None
  */
void DMA2_Channel5_IRQHandler()
{
  if (DMA2->ISR & DMA_ISR_TCIF5) {
	  DMA2->IFCR = DMA_IFCR_CTCIF5;	
    }
}
#endif

#ifdef DMA2_Channel6
/**
  * @brief  DMA2 Channel6 IRQHandler
  * @param  None
  * @retval None
  */
void DMA2_Channel6_IRQHandler()
{
  if (DMA2->ISR & DMA_ISR_TCIF6) {
	  DMA2->IFCR = DMA_IFCR_CTCIF6;	
    }
}
#endif

#ifdef DMA2_Channel7
/**
  * @brief  DMA2 Channel7 IRQHandler
  * @param  None
  * @retval None
  */
void DMA2_Channel7_IRQHandler()
{
  if (DMA2->ISR & DMA_ISR_TCIF7) {
	  DMA2->IFCR = DMA_IFCR_CTCIF7;	
    }
}
#endif

#ifdef DMA2_Channel8
/**
  * @brief  DMA2 Channel8 IRQHandler
  * @param  None
  * @retval None
  */
void DMA2_Channel8_IRQHandler()
{
  if (DMA2->ISR & DMA_ISR_TCIF8) {
	  DMA2->IFCR = DMA_IFCR_CTCIF8;	
    }
}
#endif


/* Combined handlers
DMA1_Ch2_3_DMA2_Ch1_2_IRQn
DMA1_Ch4_7_DMA2_Ch3_5_IRQn
DMA1_Channel2_3_IRQn
DMA1_Channel4_5_6_7_IRQn
DMA1_Channel4_5_IRQn
DMA2_Channel4_5_IRQn
*/

#if defined(DMA1_Channel2) && defined(DMA1_Channel3) && defined(DMA2_Channel1) && defined(DMA2_Channel2)
/**
  * @brief  DMA1 and DMA2 hander for channels 2,3 and 1,2
  * @param  None
  * @retval None
  */
void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler()
{
   if (DMA1->ISR & DMA_ISR_TCIF2) {
	  DMA1->IFCR = DMA_IFCR_CTCIF2;	
    }

   if (DMA1->ISR & DMA_ISR_TCIF3) {
	  DMA1->IFCR = DMA_IFCR_CTCIF3;	
    }

   if (DMA2->ISR & DMA_ISR_TCIF1) {
	  DMA2->IFCR = DMA_IFCR_CTCIF1;	
    }

   if (DMA2->ISR & DMA_ISR_TCIF2) {
	  DMA2->IFCR = DMA_IFCR_CTCIF2;	
    }
}
#endif

#ifdef DMA1_Stream0
void DMA1_Stream0_IRQHandler()
{
  uint32_t isr = DMA1->LISR;
	if (isr & DMA_LISR_TCIF0) {
	  DMA1->LIFCR = DMA_LIFCR_CTCIF0;
	
    }
}
#endif

#ifdef DMA1_Stream1
void DMA1_Stream1_IRQHandler()
{
  uint32_t isr = DMA1->LISR;
	if (isr & DMA_LISR_TCIF1) {
	  DMA1->LIFCR = DMA_LIFCR_CTCIF1;
	
    }
}
#endif

#ifdef DMA1_Stream2
void DMA1_Stream2_IRQHandler()
{
  uint32_t isr = DMA1->LISR;
	if (isr & DMA_LISR_TCIF2) {
	  DMA1->LIFCR = DMA_LIFCR_CTCIF2;
	
    }
}
#endif

#ifdef DMA1_Stream3
void DMA1_Stream3_IRQHandler()
{
  uint32_t isr = DMA1->LISR;
	if (isr & DMA_LISR_TCIF3) {
	  DMA1->LIFCR = DMA_LIFCR_CTCIF3;
	
    }
}
#endif

#ifdef DMA1_Stream4
void DMA1_Stream4_IRQHandler()
{
  uint32_t isr = DMA1->HISR;
	if (isr & DMA_HISR_TCIF4) {
	  DMA1->HIFCR = DMA_HIFCR_CTCIF4;
	
    }
}
#endif

#ifdef DMA1_Stream5
void DMA1_Stream5_IRQHandler()
{
  uint32_t isr = DMA1->HISR;
	if (isr & DMA_HISR_TCIF5) {
	  DMA1->HIFCR = DMA_HIFCR_CTCIF5;
	
    }
}
#endif

#ifdef DMA1_Stream6
void DMA1_Stream6_IRQHandler()
{
  uint32_t isr = DMA1->HISR;
	if (isr & DMA_HISR_TCIF6) {
	  DMA1->HIFCR = DMA_HIFCR_CTCIF6;
	
    }
}
#endif

#ifdef DMA1_Stream7
void DMA1_Stream7_IRQHandler()
{
  uint32_t isr = DMA1->HISR;
	if (isr & DMA_HISR_TCIF7) {
	  DMA1->HIFCR = DMA_HIFCR_CTCIF7;
	
    }
}
#endif

#ifdef DMA2_Stream0
void DMA2_Stream0_IRQHandler()
{
  uint32_t isr = DMA2->LISR;
	if (isr & DMA_LISR_TCIF0) {
	  DMA2->LIFCR = DMA_LIFCR_CTCIF0;
	
    }
}
#endif

#ifdef DMA2_Stream1
void DMA2_Stream1_IRQHandler()
{
  uint32_t isr = DMA2->LISR;
	if (isr & DMA_LISR_TCIF1) {
	  DMA2->LIFCR = DMA_LIFCR_CTCIF1;
	
    }
}
#endif

#ifdef DMA2_Stream2
void DMA2_Stream2_IRQHandler()
{
  uint32_t isr = DMA2->LISR;
	if (isr & DMA_LISR_TCIF3) {
	  DMA2->LIFCR = DMA_LIFCR_CTCIF3;
	
    }
}
#endif

#ifdef DMA2_Stream3
void DMA2_Stream3_IRQHandler()
{
  uint32_t isr = DMA2->LISR;
	if (isr & DMA_LISR_TCIF3) {
	  DMA2->LIFCR = DMA_LIFCR_CTCIF3;
	
    }
}
#endif

#ifdef DMA2_Stream4
void DMA2_Stream4_IRQHandler()
{
  uint32_t isr = DMA2->HISR;
	if (isr & DMA_HISR_TCIF4) {
	  DMA2->HIFCR = DMA_HIFCR_CTCIF4;
	
    }
}
#endif

#ifdef DMA2_Stream5
void DMA2_Stream5_IRQHandler()
{
  uint32_t isr = DMA2->HISR;
	if (isr & DMA_HISR_TCIF5) {
	  DMA2->HIFCR = DMA_HIFCR_CTCIF5;
	
    }
}
#endif

#ifdef DMA2_Stream6
void DMA2_Stream6_IRQHandler()
{
  uint32_t isr = DMA2->HISR;
	if (isr & DMA_HISR_TCIF6) {
	  DMA2->HIFCR = DMA_HIFCR_CTCIF6;
	
    }
}
#endif

#ifdef DMA2_Stream7
void DMA2_Stream7_IRQHandler()
{

	uint32_t isr = DMA2->HISR;
	if (isr & DMA_HISR_TCIF7) {
	  DMA2->HIFCR = DMA_HIFCR_CTCIF7;
	
    }	

}
#endif

}


#endif /* HAL_DMA_MODULE_ENABLED */


//////////////////////////////////////////////////////////////////////////////////
//Interrupts
 const IRQn_Type GPIO_IRQn[] = {
  EXTI0_IRQn,     //0
  EXTI1_IRQn,     //1
  EXTI2_IRQn,     //2
  EXTI3_IRQn,     //3
  EXTI4_IRQn,     //4
  EXTI9_5_IRQn,   //5
  EXTI9_5_IRQn,   //6
  EXTI9_5_IRQn,   //7
  EXTI9_5_IRQn,   //8
  EXTI9_5_IRQn,   //9
  EXTI15_10_IRQn, //10
  EXTI15_10_IRQn, //11
  EXTI15_10_IRQn, //12
  EXTI15_10_IRQn, //13
  EXTI15_10_IRQn, //14
  EXTI15_10_IRQn  //15
 };

typedef void (*interruptCB)(void);

static interruptCB callbacksEXTI[16];//EXTI line:0~15

/* Configure PIO interrupt sources */
static void __initialize() {
	uint16_t i;
	for (i=0; i<16; i++) {
		callbacksEXTI[i] = NULL;
	}

}



void attachInterrupt(uint32_t pin, void (*callback)(void), uint32_t mode)
{
	//uint8_t portsource = 0;	//variable to hold the port number
	uint32_t GPIO_PinSource = 0;	//variable to hold the pin number
	uint16_t PinNumber;				//temp variable to calculate the pin number
	//uint32_t gpio_clock;


//	GPIO_TypeDef *gpio_port = g_APinDescription[pin].pPort;
	uint16_t gpio_pin = g_APinDescription[pin].ulPin;
	
	static int enabled = 0;
	if (!enabled) {
		__initialize();
		enabled = 1;
	}

	pinMode(pin, mode);

	//Find out the pin number from the mask
	PinNumber = gpio_pin;
	PinNumber = PinNumber >> 1;
	while(PinNumber)
	{
		PinNumber = PinNumber >> 1;
		GPIO_PinSource++;
	}

	// Register the handler for the user function name
  	callbacksEXTI[GPIO_PinSource] = callback;
	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(GPIO_IRQn[GPIO_PinSource], 0, 0);
  HAL_NVIC_EnableIRQ(GPIO_IRQn[GPIO_PinSource]);
	
		
}


void detachInterrupt(uint32_t pin)
{
	
    uint8_t GPIO_PinSource = 0;	//variable to hold the pin number
	uint32_t PinNumber;				//temp variable to calculate the pin number

	uint16_t gpio_pin = g_APinDescription[pin].ulPin;



	//Find out the pin number from the mask
	PinNumber = gpio_pin;
	PinNumber = PinNumber >> 1;
	while(PinNumber)
	{
		PinNumber = PinNumber >> 1;
		GPIO_PinSource++;
	}

	//Clear the pending interrupt flag for that interrupt pin
	//EXTI_ClearITPendingBit(gpio_pin);
	__HAL_GPIO_EXTI_CLEAR_IT(gpio_pin);
    EXTI->IMR &= ~(1 << GPIO_PinSource);
	//unregister the user's handler
    callbacksEXTI[GPIO_PinSource] = NULL;
      
        
}

void interrupts(void){
	//Only enable the interrupts that are exposed to the user
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void noInterrupts(void){
	NVIC_DisableIRQ(EXTI0_IRQn);
	NVIC_DisableIRQ(EXTI1_IRQn);
	NVIC_DisableIRQ(EXTI2_IRQn);
	NVIC_DisableIRQ(EXTI3_IRQn);
	NVIC_DisableIRQ(EXTI4_IRQn);
	NVIC_DisableIRQ(EXTI9_5_IRQn);
	NVIC_DisableIRQ(EXTI15_10_IRQn);
}

/* interrupt handler for PA0,PB0,PC0,PD0,PE0 */
extern "C" void EXTI0_IRQHandler      (void) {

	/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_0);
			if(NULL != callbacksEXTI[0]) callbacksEXTI[0]();
			
    } 

}

/* interrupt handler for PA1,PB1,PC1,PD1,PE1 */
extern "C" void EXTI1_IRQHandler    (void) {

/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_1);
			if(NULL != callbacksEXTI[1]) callbacksEXTI[1]();
			
    }
}

/* interrupt handler for PA2,PB2,PC2,PD2,PE2 */
extern "C" void EXTI2_IRQHandler    (void) {

  /* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_2);
			if(NULL != callbacksEXTI[2]) callbacksEXTI[2]();
			
    }
}

/* interrupt handler for PA3,PB3,PC3,PD3,PE3 */
extern "C" void EXTI3_IRQHandler    (void) {

 /* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_3);
			if(NULL != callbacksEXTI[3]) callbacksEXTI[3]();
			
    }
}

/* interrupt handler for PA4,PB4,PC4,PD4,PE4 */
extern "C" void EXTI4_IRQHandler    (void) {

/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_4);
			if(NULL != callbacksEXTI[4]) callbacksEXTI[4]();
			
    }
}

/* interrupt handler for PA5~9,PB5~9,PC5~9,PD5~9,PE5~9 */
extern "C" void EXTI9_5_IRQHandler  (void) {

 
	/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_5);
			if(NULL != callbacksEXTI[5]) callbacksEXTI[5]();
			
    }

	/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_6) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_6);
			if(NULL != callbacksEXTI[6]) callbacksEXTI[6]();
			
    }

	/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_7) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_7);
			if(NULL != callbacksEXTI[7]) callbacksEXTI[7]();
			
    }

	/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_8) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_8);
			if(NULL != callbacksEXTI[8]) callbacksEXTI[8]();
			
    }

	/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_9) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_9);
			if(NULL != callbacksEXTI[9]) callbacksEXTI[9]();
			
    }
}

/* interrupt handler for PA10~15,PB10~15,PC10~15,PD1015,PE10~15 */
extern "C" void EXTI15_10_IRQHandler(void) {
 

	/* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_10);
			if(NULL != callbacksEXTI[10]) callbacksEXTI[10]();
			
    }
	 /* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_11);
			if(NULL != callbacksEXTI[11]) callbacksEXTI[11]();
			
    }
	 /* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_12) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_12);
			if(NULL != callbacksEXTI[12]) callbacksEXTI[12]();
			
    }
	 /* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_13);
			if(NULL != callbacksEXTI[13]) callbacksEXTI[13]();
			
    }
	 /* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_14);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_14);
			if(NULL != callbacksEXTI[14]) callbacksEXTI[14]();
			
    }
	 /* EXTI line interrupt detected */
   if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET)
   {
      __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
      HAL_GPIO_EXTI_Callback(GPIO_PIN_15);
			if(NULL != callbacksEXTI[15]) callbacksEXTI[15]();
			
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
extern TIM_HandleTypeDef htim2;
__IO uint32_t TIM2_TimingMillis;

	 uint32_t msec;

	tKind_t type;
  uint8_t num;
	uint8_t conf_flag;

void timer_handler(void);

extern TimerManager_t TManager;



//extern "C" void TIM2_IRQHandler(void)
//{
//  HAL_TIM_IRQHandler(&htim2);
//	
////  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
////  {
////   
////     TIM2_TimingMillis++;
////     timer_handler();
////     TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
////	 
////  }
//}
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance==htim2.Instance)
//	{
//		TIM2_TimingMillis++;
//     timer_handler();
//	}
//}


TIM_HandleTypeDef htim2;
void TIM2_Configuration(void)
{
  
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	 /* NVIC Configuration */
//	NVIC_InitTypeDef NVIC_InitStructure;
	//TIM_HandleTypeDef  TIMx;
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
   //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    __HAL_RCC_TIM2_CLK_ENABLE();  
      
    
  // RCC_Configuration();

 

  /* Enable the TIM7 global Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
	HAL_NVIC_SetPriority(TIM2_IRQn, 6, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
  
 
  
//  TIM_TimeBaseStructure.TIM_Period = 1000;
//  TIM_TimeBaseStructure.TIM_Prescaler = 0;
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		
		htim2.Instance =TIM2;
			// Time base configuration
			htim2.Init.Period = 1000;
			htim2.Init.Prescaler = 0;
			htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//0
			htim2.Init.CounterMode = TIM_COUNTERMODE_UP;			
			//HAL_TIM_Base_Init(&htim2);
			
			 if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
				{
					Error_Handler();
				}

				sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
				if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
				{
					Error_Handler();
				}

				sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
				sMasterConfig.MasterSlaveMode = 
				TIM_MASTERSLAVEMODE_DISABLE;
				if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
				{
					Error_Handler();
				}

  /* Prescaler configuration */
  //TIM_PrescalerConfig(TIM2, 83, TIM_PSCReloadMode_Immediate);
				__HAL_TIM_SET_PRESCALER(&htim2, 83);

 
//  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//  /* TIM IT enable */  
//  TIM_ITConfig(TIM2, TIM_IT_Update , ENABLE);
//  
//  /* TIM7 enable counter */
//  TIM_Cmd(TIM2, ENABLE);
				
				HAL_TIM_Base_Start_IT(&htim2);
	
	
		
		
		

  
}



void timer_handler(void)
{
       Node_t *tail , *temp;
	   //aabb=12;

       if (TManager.Head.prev != NULL)
       {
            for (tail = TManager.Head.next; tail != &TManager.Head; tail = temp)
            {
                    temp = tail->next;
                    //aabb++;
                    if (TIM2_TimingMillis - tail->Timer->start == tail->Timer->msec && tail->Timer->flag == 1)
                    {
                            tail->Timer->tFunc(tail->Timer->data);
                            if (tail->Timer->type == t_single)
                            {
                                    tail->Timer->flag = 0;
                                    //Timer_delete(tail->Timer->num);
                                    Timer_delete(tail->Timer->num);
                            }
                            else
                            {
                                    tail->Timer->start = TIM2_TimingMillis;
                            }
                    }
            
            
            }
       }







}


uint8_t Timer_Add(uint32_t ms, Func tfunc, tKind_t type, void *data)
{

	Timer_t *timer;
	Node_t *newtimer;
	Node_t *tail;

	if (TManager.TotalNum == 0)
	{
		   
	      
		TManager.Head.prev = &TManager.Head;
		TManager.Head.next = &TManager.Head;
		 
        TIM2_Configuration();
	

	}
    
    timer = (Timer_t *)malloc(sizeof(Timer_t));
	if (timer == NULL)
		return -1;
	

	timer->msec = ms;
	timer->tFunc = tfunc;
	timer->type = type;
	timer->data = data;
    timer->start = TIM2_TimingMillis;
	timer->num = TManager.TotalNum;
    timer->flag = 1;
    newtimer = (Node_t *)malloc(sizeof(Node_t));
    if (timer == NULL)
		return -1;
	
    newtimer->Timer = timer;
	for (tail = &TManager.Head; tail->next != &TManager.Head; tail = tail ->next)
	{
		
	
	}

	newtimer->next = &TManager.Head;
    newtimer->prev = tail;

	tail->next = newtimer;
	TManager.Head.prev = newtimer;
    TManager.TotalNum++;

	return  timer->num;
	

}

void Timer_delete(uint8_t Num)
{
    int i = 0;
    Node_t *tail, *temp;
  
    
     for (tail = TManager.Head.next; tail != &TManager.Head; tail = temp)
     {

	 
	
          temp = tail->next;
		 
          if (tail->Timer->num == Num)
          {
           
              temp->prev = tail->prev;
			     
              tail->prev->next = temp;
          
              if(tail->Timer->data != NULL)
                free(tail->Timer->data);
              free(tail->Timer);
              free(tail);
              TManager.TotalNum--;
          }
     }
	
    
}


uint32_t get_resitime(uint8_t num)
{
	 Node_t *tail;
    
    
     for (tail = TManager.Head.next; tail != &TManager.Head; tail = tail->next)
     {
     	if (tail->Timer->num == num)
     	{
			return tail->Timer->msec - (TIM2_TimingMillis - tail->Timer->start);
		}


	 }
	

}

void change_cbFunc(uint8_t num, Func ttFunc)
{
	Node_t *tail;
	   
	   
		for (tail = TManager.Head.next; tail != &TManager.Head; tail = tail->next)
		{
		   if (tail->Timer->num == num)
		   {
		   	  tail->Timer->tFunc = ttFunc;
			  return;
		   }
	
	
		}


}


void change_ttype(uint8_t num, tKind_t type)
{

		Node_t *tail;
	   
	   
		for (tail = TManager.Head.next; tail != &TManager.Head; tail = tail->next)
		{
		   if (tail->Timer->num == num)
		   {
		   	  tail->Timer->type = type;
			  return;
		   }
	
	
		}



}

////////////////////////////////////////////////////////

void Timer(uint32_t tms, Func tfunc, tKind_t mode, void *tdata)
{
	conf_flag = 0;
	//asm("CPSIE	I");   //
	__enable_irq();//INT_ENABLE();

	num = Timer_Add(tms, tfunc, mode, tdata);
	
	msec = tms;
	
	type = mode;
	
    conf_flag = 1;

}



//Timer::~Timer()
//{
//	

//   
//   Timer_delete(num);
//  
//}

void Timer_config(uint32_t tms, Func tfunc, tKind_t mode, void *tdata)
{
    if (!conf_flag)
   	{
   	    
		num = Timer_Add(tms, tfunc, mode, tdata);
		msec = tms;
	
		type = mode;
	
    }
	
	conf_flag = 1;



}





uint8_t Timer_get_tNum(void)
{

		return num;

}





uint32_t Timer_get_resTime(void)
{


 	return get_resitime(num);
	



}



void Timer_change_callbackFunc(Func ttFunc)
{

	
	change_cbFunc(num, ttFunc);
}


void Timer_change_type( tKind_t mode)
{

    change_ttype(num, mode);

}


///////////////////////////////////////////////////////////////////////////////////////////
extern void randomSeed( uint32_t dwSeed )
{
  if ( dwSeed != 0 )
  {
    srand( dwSeed ) ;
  }
}

//extern long random( long howbig )
//{
//  if ( howbig == 0 )
//  {
//    return 0 ;
//  }

//  return rand() % howbig;
//}

extern long random( long howsmall, long howbig )
{
  if (howsmall >= howbig)
  {
    return howsmall;
  }

  long diff = howbig - howsmall;
	
  if ( diff == 0 )
  {
    return 0 ;
  }
 

  return (rand() % diff) + howsmall;
}


extern uint16_t makeWord( uint16_t w )
{
  return w ;
}

//extern uint16_t makeWord( uint8_t h, uint8_t l )
//{
//  return (h << 8) | l ;
//}
/////////////////////////////////////////////////////////////////////////////////////

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
////////////////////////////////////////////////////////////
uint32_t shiftIn( uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder )
{
	uint8_t value = 0 ;
	uint8_t i ;
	
//	GPIO_Set(GPIOx,ulDataPin,GPIO_MODE_IN,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
//	GPIO_Set(GPIOx,ulClockPin,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
	pinMode(ulDataPin,INPUT_PULLUP);
	pinMode(ulClockPin,OUTPUT);
	
	for ( i=0 ; i < 8 ; ++i )
    {
		digitalWrite( ulClockPin, 1 ) ;

		if ( ulBitOrder == LSBFIRST )
        {
			value |= digitalRead( ulDataPin ) << i ;
        }
		else
        {
			value |= digitalRead( ulDataPin ) << (7 - i) ;
        }

		digitalWrite( ulClockPin, 0 ) ;
	}

	return value ;
}

void shiftOut(uint32_t ulDataPin, uint32_t ulClockPin, uint32_t ulBitOrder, uint32_t ulVal )
{
	uint8_t i ;
	//GPIO_Set(GPIOx,ulDataPin,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
	//GPIO_Set(GPIOx,ulClockPin,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
	pinMode(ulDataPin,OUTPUT);
	pinMode(ulClockPin,OUTPUT);
	for ( i=0 ; i < 8 ; i++ )
    {
		if ( ulBitOrder == LSBFIRST )
        {
			digitalWrite( ulDataPin, !!(ulVal & (1 << i)) ) ;
        }
		else	
        {
			digitalWrite( ulDataPin, !!(ulVal & (1 << (7 - i))) ) ;
        }

		digitalWrite(  ulClockPin, 1 ) ;
		digitalWrite(  ulClockPin, 0 ) ;		
	}
}
//////////////////////////////////////////////////////////////
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}
//////////////////////////////////////////////

//////////////////////////////////////////////////////////

#if 0
/* reverse:  reverse string s in place */
static void reverse( char s[] )
{
  int i, j ;
  char c ;

  for ( i = 0, j = strlen(s)-1 ; i < j ; i++, j-- )
  {
    c = s[i] ;
    s[i] = s[j] ;
    s[j] = c ;
  }
}

/* itoa:  convert n to characters in s */
extern void itoa( int n, char s[] )
{
  int i, sign ;

  if ( (sign = n) < 0 )  /* record sign */
  {
    n = -n;          /* make n positive */
  }

  i = 0;
  do
  {       /* generate digits in reverse order */
    s[i++] = n % 10 + '0';   /* get next digit */
  } while ((n /= 10) > 0) ;     /* delete it */

  if (sign < 0 )
  {
    s[i++] = '-';
  }

  s[i] = '\0';

  reverse( s ) ;
}

#else

extern char* itoa( int value, char *string, int radix )
{
  return ltoa( value, string, radix ) ;
}

extern char* ltoa( long value, char *string, int radix )
{
  char tmp[33];
  char *tp = tmp;
  long i;
  unsigned long v;
  int sign;
  char *sp;

  if ( string == NULL )
  {
    return 0 ;
  }

  if (radix > 36 || radix <= 1)
  {
    return 0 ;
  }

  sign = (radix == 10 && value < 0);
  if (sign)
  {
    v = -value;
  }
  else
  {
    v = (unsigned long)value;
  }

  while (v || tp == tmp)
  {
    i = v % radix;
    v = v / radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }

  sp = string;

  if (sign)
    *sp++ = '-';
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;

  return string;
}

extern char* utoa( unsigned long value, char *string, int radix )
{
  return ultoa( value, string, radix ) ;
}

extern char* ultoa( unsigned long value, char *string, int radix )
{
  char tmp[33];
  char *tp = tmp;
  long i;
  unsigned long v = value;
  char *sp;

  if ( string == NULL )
  {
    return 0;
  }

  if (radix > 36 || radix <= 1)
  {
    return 0;
  }
 
  while (v || tp == tmp)
  {
    i = v % radix;
    v = v / radix;
    if (i < 10)
      *tp++ = i+'0';
    else
      *tp++ = i + 'a' - 10;
  }

  sp = string;

 
  while (tp > tmp)
    *sp++ = *--tp;
  *sp = 0;

  return string;
}
#endif /* 0 */
///////////////////////////////////////////////////////////////////////////////////////////////////////
#include "IString.h"
//#include "itoa.h"//trong sys.h

/*********************************************/
/*  Constructors                             */
/*********************************************/

String::String(const char *cstr)
{
	init();
	if (cstr) copy(cstr, strlen(cstr));
}

String::String(const String &value)
{
	init();
	*this = value;
}

String::String(const __FlashStringHelper *pstr)
{
	init();
	*this = pstr;
}

#ifdef __GXX_EXPERIMENTAL_CXX0X__
String::String(String &&rval)
{
	init();
	move(rval);
}
String::String(StringSumHelper &&rval)
{
	init();
	move(rval);
}
#endif

String::String(char c)
{
	init();
	char buf[2];
	buf[0] = c;
	buf[1] = 0;
	*this = buf;
}

String::String(unsigned char value, unsigned char base)
{
	init();
	char buf[9];
	utoa(value, buf, base);
	*this = buf;
}

String::String(int value, unsigned char base)
{
	init();
	char buf[18];
	itoa(value, buf, base);
	*this = buf;
}

String::String(unsigned int value, unsigned char base)
{
	init();
	char buf[17];
	utoa(value, buf, base);
	*this = buf;
}

String::String(long value, unsigned char base)
{
	init();
	char buf[34];
	ltoa(value, buf, base);
	*this = buf;
}

String::String(unsigned long value, unsigned char base)
{
	init();
	char buf[33];
	ultoa(value, buf, base);
	*this = buf;
}

String::String(float value, unsigned char decimalPlaces)
{
	init();
	char buf[33];
	*this = dtostrf(value, (decimalPlaces + 2), decimalPlaces, buf);
}

String::String(double value, unsigned char decimalPlaces)
{
	init();
	char buf[33];
	*this = dtostrf(value, (decimalPlaces + 2), decimalPlaces, buf);
}

String::~String()
{
	free(buffer);
}

/*********************************************/
/*  Memory Management                        */
/*********************************************/

inline void String::init(void)
{
	buffer = NULL;
	capacity = 0;
	len = 0;
}

void String::invalidate(void)
{
	if (buffer) free(buffer);
	buffer = NULL;
	capacity = len = 0;
}

unsigned char String::reserve(unsigned int size)
{
	if (buffer && capacity >= size) return 1;
	if (changeBuffer(size)) {
		if (len == 0) buffer[0] = 0;
		return 1;
	}
	return 0;
}

unsigned char String::changeBuffer(unsigned int maxStrLen)
{
	char *newbuffer = (char *)realloc(buffer, maxStrLen + 1);
	if (newbuffer) {
		buffer = newbuffer;
		capacity = maxStrLen;
		return 1;
	}
	return 0;
}

/*********************************************/
/*  Copy and Move                            */
/*********************************************/

String & String::copy(const char *cstr, unsigned int length)
{
	if (!reserve(length)) {
		invalidate();
		return *this;
	}
	len = length;
	strcpy(buffer, cstr);
	return *this;
}

String & String::copy(const __FlashStringHelper *pstr, unsigned int length)
{
	if (!reserve(length)) {
		invalidate();
		return *this;
	}
	len = length;
	strcpy_P(buffer, (const prog_char *)pstr);
	return *this;
}

#ifdef __GXX_EXPERIMENTAL_CXX0X__
void String::move(String &rhs)
{
	if (buffer) {
		if (capacity >= rhs.len) {
			strcpy(buffer, rhs.buffer);
			len = rhs.len;
			rhs.len = 0;
			return;
		} else {
			free(buffer);
		}
	}
	buffer = rhs.buffer;
	capacity = rhs.capacity;
	len = rhs.len;
	rhs.buffer = NULL;
	rhs.capacity = 0;
	rhs.len = 0;
}
#endif

String & String::operator = (const String &rhs)
{
	if (this == &rhs) return *this;
	
	if (rhs.buffer) copy(rhs.buffer, rhs.len);
	else invalidate();
	
	return *this;
}

#ifdef __GXX_EXPERIMENTAL_CXX0X__
String & String::operator = (String &&rval)
{
	if (this != &rval) move(rval);
	return *this;
}

String & String::operator = (StringSumHelper &&rval)
{
	if (this != &rval) move(rval);
	return *this;
}
#endif

String & String::operator = (const char *cstr)
{
	if (cstr) copy(cstr, strlen(cstr));
	else invalidate();
	
	return *this;
}

String & String::operator = (const __FlashStringHelper *pstr)
{
	if (pstr) copy(pstr, strlen_P((const prog_char *)pstr));
	else invalidate();

	return *this;
}

/*********************************************/
/*  concat                                   */
/*********************************************/

unsigned char String::concat(const String &s)
{
	return concat(s.buffer, s.len);
}

unsigned char String::concat(const char *cstr, unsigned int length)
{
	unsigned int newlen = len + length;
	if (!cstr) return 0;
	if (length == 0) return 1;
	if (!reserve(newlen)) return 0;
	strcpy(buffer + len, cstr);
	len = newlen;
	return 1;
}

unsigned char String::concat(const char *cstr)
{
	if (!cstr) return 0;
	return concat(cstr, strlen(cstr));
}

unsigned char String::concat(char c)
{
	char buf[2];
	buf[0] = c;
	buf[1] = 0;
	return concat(buf, 1);
}

unsigned char String::concat(unsigned char num)
{
	char buf[4];
	itoa(num, buf, 10);
	return concat(buf, strlen(buf));
}

unsigned char String::concat(int num)
{
	char buf[12];
	itoa(num, buf, 10);
	return concat(buf, strlen(buf));
}

unsigned char String::concat(unsigned int num)
{
	char buf[11];
	utoa(num, buf, 10);
	return concat(buf, strlen(buf));
}

unsigned char String::concat(long num)
{
	char buf[12];
	ltoa(num, buf, 10);
	return concat(buf, strlen(buf));
}

unsigned char String::concat(unsigned long num)
{
	char buf[11];
	ultoa(num, buf, 10);
	return concat(buf, strlen(buf));
}

unsigned char String::concat(float num)
{
	char buf[20];
	char* string = dtostrf(num, 4, 2, buf);
	return concat(string, strlen(string));
}

unsigned char String::concat(double num)
{
	char buf[20];
	char* string = dtostrf(num, 4, 2, buf);
	return concat(string, strlen(string));
}

unsigned char String::concat(const __FlashStringHelper * str)
{
	if (!str) return 0;
	int length = strlen_P((const char *) str);
	if (length == 0) return 1;
	unsigned int newlen = len + length;
	if (!reserve(newlen)) return 0;
	strcpy_P(buffer + len, (const char *) str);
	len = newlen;
	return 1;
}

/*********************************************/
/*  Concatenate                              */
/*********************************************/

StringSumHelper & operator + (const StringSumHelper &lhs, const String &rhs)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(rhs.buffer, rhs.len)) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, const char *cstr)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!cstr || !a.concat(cstr, strlen(cstr))) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, char c)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(c)) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, unsigned char num)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(num)) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, int num)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(num)) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, unsigned int num)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(num)) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, long num)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(num)) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, unsigned long num)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(num)) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, float num)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(num)) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, double num)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(num)) a.invalidate();
	return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, const __FlashStringHelper *rhs)
{
	StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
	if (!a.concat(rhs))	a.invalidate();
	return a;
}

/*********************************************/
/*  Comparison                               */
/*********************************************/

int String::compareTo(const String &s) const
{
	if (!buffer || !s.buffer) {
		if (s.buffer && s.len > 0) return 0 - *(unsigned char *)s.buffer;
		if (buffer && len > 0) return *(unsigned char *)buffer;
		return 0;
	}
	return strcmp(buffer, s.buffer);
}

unsigned char String::equals(const String &s2) const
{
	return (len == s2.len && compareTo(s2) == 0);
}

unsigned char String::equals(const char *cstr) const
{
	if (len == 0) return (cstr == NULL || *cstr == 0);
	if (cstr == NULL) return buffer[0] == 0;
	return strcmp(buffer, cstr) == 0;
}

unsigned char String::operator<(const String &rhs) const
{
	return compareTo(rhs) < 0;
}

unsigned char String::operator>(const String &rhs) const
{
	return compareTo(rhs) > 0;
}

unsigned char String::operator<=(const String &rhs) const
{
	return compareTo(rhs) <= 0;
}

unsigned char String::operator>=(const String &rhs) const
{
	return compareTo(rhs) >= 0;
}

unsigned char String::equalsIgnoreCase( const String &s2 ) const
{
	if (this == &s2) return 1;
	if (len != s2.len) return 0;
	if (len == 0) return 1;
	const char *p1 = buffer;
	const char *p2 = s2.buffer;
	while (*p1) {
		if (tolower(*p1++) != tolower(*p2++)) return 0;
	} 
	return 1;
}

unsigned char String::startsWith( const String &s2 ) const
{
	if (len < s2.len) return 0;
	return startsWith(s2, 0);
}

unsigned char String::startsWith( const String &s2, unsigned int offset ) const
{
	if (offset > len - s2.len || !buffer || !s2.buffer) return 0;
	return strncmp( &buffer[offset], s2.buffer, s2.len ) == 0;
}

unsigned char String::endsWith( const String &s2 ) const
{
	if ( len < s2.len || !buffer || !s2.buffer) return 0;
	return strcmp(&buffer[len - s2.len], s2.buffer) == 0;
}

/*********************************************/
/*  Character Access                         */
/*********************************************/

char String::charAt(unsigned int loc) const
{
	return operator[](loc);
}

void String::setCharAt(unsigned int loc, char c) 
{
	if (loc < len) buffer[loc] = c;
}

char & String::operator[](unsigned int index)
{
	static char dummy_writable_char;
	if (index >= len || !buffer) {
		dummy_writable_char = 0;
		return dummy_writable_char;
	}
	return buffer[index];
}

char String::operator[]( unsigned int index ) const
{
	if (index >= len || !buffer) return 0;
	return buffer[index];
}

void String::getBytes(unsigned char *buf, unsigned int bufsize, unsigned int index) const
{
	if (!bufsize || !buf) return;
	if (index >= len) {
		buf[0] = 0;
		return;
	}
	unsigned int n = bufsize - 1;
	if (n > len - index) n = len - index;
	strncpy((char *)buf, buffer + index, n);
	buf[n] = 0;
}

/*********************************************/
/*  Search                                   */
/*********************************************/

int String::indexOf(char c) const
{
	return indexOf(c, 0);
}

int String::indexOf( char ch, unsigned int fromIndex ) const
{
	if (fromIndex >= len) return -1;
	const char* temp = strchr(buffer + fromIndex, ch);
	if (temp == NULL) return -1;
	return temp - buffer;
}

int String::indexOf(const String &s2) const
{
	return indexOf(s2, 0);
}

int String::indexOf(const String &s2, unsigned int fromIndex) const
{
	if (fromIndex >= len) return -1;
	const char *found = strstr(buffer + fromIndex, s2.buffer);
	if (found == NULL) return -1;
	return found - buffer;
}

int String::lastIndexOf( char theChar ) const
{
	return lastIndexOf(theChar, len - 1);
}

int String::lastIndexOf(char ch, unsigned int fromIndex) const
{
	if (fromIndex >= len) return -1;
	char tempchar = buffer[fromIndex + 1];
	buffer[fromIndex + 1] = '\0';
	char* temp = strrchr( buffer, ch );
	buffer[fromIndex + 1] = tempchar;
	if (temp == NULL) return -1;
	return temp - buffer;
}

int String::lastIndexOf(const String &s2) const
{
	return lastIndexOf(s2, len - s2.len);
}

int String::lastIndexOf(const String &s2, unsigned int fromIndex) const
{
  	if (s2.len == 0 || len == 0 || s2.len > len) return -1;
	if (fromIndex >= len) fromIndex = len - 1;
	int found = -1;
	for (char *p = buffer; p <= buffer + fromIndex; p++) {
		p = strstr(p, s2.buffer);
		if (!p) break;
		if ((unsigned int)(p - buffer) <= fromIndex) found = p - buffer;
	}
	return found;
}

String String::substring(unsigned int left, unsigned int right) const
{
	if (left > right) {
		unsigned int temp = right;
		right = left;
		left = temp;
	}
	String out;
	if (left > len) return out;
	if (right > len) right = len;
	char temp = buffer[right];  // save the replaced character
	buffer[right] = '\0';	
	out = buffer + left;  // pointer arithmetic
	buffer[right] = temp;  //restore character
	return out;
}

/*********************************************/
/*  Modification                             */
/*********************************************/

void String::replace(char find, char replace)
{
	if (!buffer) return;
	for (char *p = buffer; *p; p++) {
		if (*p == find) *p = replace;
	}
}

void String::replace(const String& find, const String& replace)
{
	if (len == 0 || find.len == 0) return;
	int diff = replace.len - find.len;
	char *readFrom = buffer;
	char *foundAt;
	if (diff == 0) {
		while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
			memcpy(foundAt, replace.buffer, replace.len);
			readFrom = foundAt + replace.len;
		}
	} else if (diff < 0) {
		char *writeTo = buffer;
		while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
			unsigned int n = foundAt - readFrom;
			memcpy(writeTo, readFrom, n);
			writeTo += n;
			memcpy(writeTo, replace.buffer, replace.len);
			writeTo += replace.len;
			readFrom = foundAt + find.len;
			len += diff;
		}
		strcpy(writeTo, readFrom);
	} else {
		unsigned int size = len; // compute size needed for result
		while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
			readFrom = foundAt + find.len;
			size += diff;
		}
		if (size == len) return;
		if (size > capacity && !changeBuffer(size)) return; // XXX: tell user!
		int index = len - 1;
		while (index >= 0 && (index = lastIndexOf(find, index)) >= 0) {
			readFrom = buffer + index + find.len;
			memmove(readFrom + diff, readFrom, len - (readFrom - buffer));
			len += diff;
			buffer[len] = 0;
			memcpy(buffer + index, replace.buffer, replace.len);
			index--;
		}
	}
}

void String::remove(unsigned int index){
	if (index >= len) { return; }
	int count = len - index;
	remove(index, count);
}

void String::remove(unsigned int index, unsigned int count){
	if (index >= len) { return; }
	if (count <= 0) { return; }
	if (index + count > len) { count = len - index; }
	char *writeTo = buffer + index;
	len = len - count;
	strncpy(writeTo, buffer + index + count,len - index);
	buffer[len] = 0;
}

void String::toLowerCase(void)
{
	if (!buffer) return;
	for (char *p = buffer; *p; p++) {
		*p = tolower(*p);
	}
}

void String::toUpperCase(void)
{
	if (!buffer) return;
	for (char *p = buffer; *p; p++) {
		*p = toupper(*p);
	}
}

void String::trim(void)
{
	if (!buffer || len == 0) return;
	char *begin = buffer;
	while (isspace(*begin)) begin++;
	char *end = buffer + len - 1;
	while (isspace(*end) && end >= begin) end--;
	len = end + 1 - begin;
	if (begin > buffer) memcpy(buffer, begin, len);
	buffer[len] = 0;
}

/*********************************************/
/*  Parsing / Conversion                     */
/*********************************************/

long String::toInt(void) const
{
	if (buffer) return atol(buffer);
	return 0;
}

float String::toFloat(void) const
{
	if (buffer) return float(atof(buffer));
	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////
static void __empty() {
	// Empty
}
void yield(void) __attribute__ ((weak, alias("__empty")));

uint32_t millis( void )
{
// todo: ensure no interrupts
	//return GetTickCount() ;
  //return TimingMillis;
	return HAL_GetTick();
}

__STATIC_INLINE uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
// Interrupt-compatible version of micros
// Theory: repeatedly take readings of SysTick counter, millis counter and SysTick interrupt pending flag.
// When it appears that millis counter and pending is stable and SysTick hasn't rolled over, use these 
// values to calculate micros. If there is a pending SysTick, add one to the millis counter in the calculation.
uint32_t micros( void )
{
   /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  LL_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (LL_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}

// original function:
// uint32_t micros( void )
// {
//     uint32_t ticks ;
//     uint32_t count ;
// 
//     SysTick->CTRL;
//     do {
//         ticks = SysTick->VAL;
//         count = GetTickCount();
//     } while (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
// 
//     return count * 1000 + (SysTick->LOAD + 1 - ticks) / (SystemCoreClock/1000000) ;
// }

void delay_ms (uint32_t nms)
{
//uint32_t Time = millis();
//while ((nms - (millis()-Time))> 0);
	delay( nms );
}

void delay_us(uint32_t nus)
{
//uint32_t Time = micros();
//while((nus-(micros()-Time)) > 0);
	delayMicroseconds(nus);
}
void delay( uint32_t ms )
{
//    uint32_t end = millis() + ms;
//    while (millis() < end)
//    	yield();
	const uint32_t count_1ms = HAL_RCC_GetSysClockFreq() / 7010;
		for (int i = 0; i < ms; i++) {
			for (uint32_t count = 0; ++count <= count_1ms;) {
			}
		}
}	
void delayMicroseconds(uint32_t nus)
{
	const uint32_t count_1us = HAL_RCC_GetSysClockFreq() / 8000000;
		for (int i = 0; i < nus; i++) {
			for (uint32_t count = 0; ++count <= count_1us;) {
			}
		}	
}











