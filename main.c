/**
	******************************************************************************
	* @file		ex6_pwm_dynamic/src/main.c
	* @author	MDS
	* @date		08032017
	* @brief	 Produces a dynamic PWM  Waveform on pin D3.
	*			 See Section 18 (TIM3), P576 of the STM32F4xx Reference Manual.
	*
	*			NOTE: Refer to lineS 3163 and 4628 of the stm32f4xx_hal_tim.c, and
    *			lines 960 and 974 of stm32f4xx_hal_tim.h files. Refer to pages
    *			102-103 of the STM32F4-Technical-Training.pdf and page 590 of the
    *			STM32F4XX_reference.pdf. It's pretty good.
	******************************************************************************
	*
	*/

#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"

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

void Hardware_init();
void set_PWM_freq(int);

TIM_HandleTypeDef TIM_Init;
TIM_OC_InitTypeDef PWMConfig;


int main(void) {

	uint32_t freq = 40000;
	char RxChar;

	/*FHSS Integers generated using matlab
	 * R = randi([20, 50], [1, 32])
	 *[sprintf('%d,', R(1:end-1)), sprintf('%d', R(end))]
	 * */
	uint8_t FHSS[32] = {28,21,23,45,41,29,49,21,33,31,43,44,25,35,33,40,41,43,28,41,40,25,23,35,49,30,38,26,43,27,35,41};
	BRD_init();
	Hardware_init();

	while (1) {

		BRD_LEDRedToggle();
		HAL_Delay(40);
		RxChar = debug_getc();
		if (RxChar != '\0') {
			freq = freq + 1000;
			set_PWM_freq(freq);	
			debug_printf("New F: %dkHz\n\r", freq/1000);
		}
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

void Hardware_init(void) {

	GPIO_InitTypeDef GPIO_InitStructure;

	uint16_t PrescalerValue = 0;

	BRD_LEDInit();
	BRD_LEDRedOff();

	PWM_PIN_CLK();
	__BRD_D3_GPIO_CLK();

	GPIO_InitStructure.Pin = PWM_PIN;
	GPIO_InitStructure.Mode =GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Alternate = PWM_GPIO_AF;
	HAL_GPIO_Init(BRD_D3_GPIO_PORT, &GPIO_InitStructure);

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

}

