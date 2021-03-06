/**
	******************************************************************************
	* @file		pwm_dynamic/main.c
	* @author	MDS, mods by ACT
	* @date		210319
	* @brief	Produces FHSS burst on pin D3 on terminal input
	* 		Constantly streams A0 data to terminal
	*			 See Section 18 (TIM3), P576 of the STM32F4xx Reference Manual.
	*		
	*			NOTE: Refer to lineS 3163 and 4628 of the stm32f4xx_hal_tim.c, and
    *			lines 960 and 974 of stm32f4xx_hal_tim.h files. Refer to pages
    *			102-103 of the STM32F4-Technical-Training.pdf and page 590 of the
    *			STM32F4XX_reference.pdf. It's pretty good.
	******************************************************************************
	*
	* Generates PWM on D3 when user button pressed. Samples analog values on A0 during
	*
	*
	*
	*
	*/

#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "stm32f4xx_hal.h"
#include "debug_printf.h"
#include "string.h"

//8 MHz
#define PWM_CLOCKFREQ		8000000
#define PWM_FREQ		40000
#define PWM_PERIOD		PWM_CLOCKFREQ/(PWM_FREQ/2)		
#define PWM_PULSEPERIOD		PWM_PERIOD/2		
#define PWM_CHANNEL			TIM_CHANNEL_3
#define PWM_PIN				BRD_D3_PIN
#define PWM_TIMER			TIM1
#define PWM_GPIO_AF			GPIO_AF1_TIM1
#define PWM_PIN_CLK()		__TIM1_CLK_ENABLE()
#define PWM_TIMER_HANDLER	TIM_Init
#define PWM_DC_GET() 		__HAL_TIM_GET_COMPARE(&PWM_TIMER_HANDLER, PWM_CHANNEL)
#define PWM_DC_SET(value) 	__HAL_TIM_SET_COMPARE(&PWM_TIMER_HANDLER, PWM_CHANNEL, value)

#define FHSS_MAX 32

void Hardware_init();
void EXTI5_10_IRQHandler(void);
void set_PWM_freq(int);

TIM_HandleTypeDef TIM_Init;
TIM_OC_InitTypeDef PWMConfig;

/*FHSS Integers generated using matlab
 * R = randi([20, 50], [1, 32])
 *[sprintf('%d,', R(1:end-1)), sprintf('%d', R(end))]
 * */	
uint8_t FHSS[FHSS_MAX] = {28,21,23,45,41,29,49,21,33,31,43,44,25,35,33,40,41,43,28,41,40,25,23,35,49,30,38,26,43,27,35,41};

int main(void) {

	BRD_init();
	Hardware_init();
	while (1) {
		;
	}

	return 1;
}

void set_PWM_freq(int frequency) {
	TIM_Init.Init.Period = PWM_CLOCKFREQ/(frequency/2);
	PWMConfig.Pulse = PWM_CLOCKFREQ/(frequency/2)/2;
			
	HAL_TIM_PWM_Init(&TIM_Init);
	HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_CHANNEL);
	HAL_TIM_PWM_Start(&TIM_Init, PWM_CHANNEL);
			

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	
	if (GPIO_Pin == GPIO_PIN_13)
	{
		for(int i = 0; i < FHSS_MAX; i++) {
			set_PWM_freq(FHSS[i]*1000);
			HAL_Delayus(200);//1ms = 8 periods @ 40kHz 
			
			HAL_TIM_PWM_Stop(&TIM_Init, PWM_CHANNEL);
			//I'm an idiot this isn't multithreaded
			HAL_Delayus(800); //1 ms total wait, 32ms (40ms with final wait) per distance measurement
		}
		HAL_TIM_PWM_Stop(&TIM_Init, PWM_CHANNEL);	
		HAL_Delayus(6000); //6ms wait, total 1.98m measure distance then. Easy to change.
	}
}

void EXTI15_10_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(BRD_USER_BUTTON_PIN);
}


void Hardware_init(void) {

	GPIO_InitTypeDef GPIO_InitStructure;
	
	uint16_t PrescalerValue = 0;

	BRD_LEDInit();
	BRD_LEDRedOff();

	PWM_PIN_CLK();
	//PWM Pin
	__BRD_D3_GPIO_CLK();

	//PWM
	GPIO_InitStructure.Pin = PWM_PIN;
	GPIO_InitStructure.Mode =GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = PWM_GPIO_AF;
	HAL_GPIO_Init(BRD_D3_GPIO_PORT, &GPIO_InitStructure);

	/*UserButton*/
	BRD_USER_BUTTON_GPIO_CLK_ENABLE();
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = BRD_USER_BUTTON_PIN;

	HAL_GPIO_Init(BRD_USER_BUTTON_GPIO_PORT, &GPIO_InitStructure);

	HAL_NVIC_SetPriority(BRD_USER_BUTTON_EXTI_IRQn, 10, 0);
	HAL_NVIC_EnableIRQ(BRD_USER_BUTTON_EXTI_IRQn);

	/*Timer & PWM Setup*/	
	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / PWM_CLOCKFREQ) - 1;

	TIM_Init.Instance = PWM_TIMER;
	TIM_Init.Init.Period = PWM_PERIOD;
	TIM_Init.Init.Prescaler = PrescalerValue;
	TIM_Init.Init.ClockDivision = 0;
	TIM_Init.Init.RepetitionCounter = 0;
	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;

	PWMConfig.OCMode = TIM_OCMODE_PWM1;
	PWMConfig.Pulse = PWM_PULSEPERIOD;
	PWMConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	PWMConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	PWMConfig.OCFastMode = TIM_OCFAST_DISABLE;
	PWMConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
	PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	HAL_TIM_PWM_Init(&TIM_Init);
	HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_CHANNEL);

	HAL_TIM_PWM_Start(&TIM_Init, PWM_CHANNEL);
	HAL_TIM_PWM_Stop(&TIM_Init, PWM_CHANNEL);

}



