#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "stm32f10x.h"


#define            ADVANCE_TIM                   TIM1
#define            ADVANCE_TIM_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define            ADVANCE_TIM_CLK               RCC_APB2Periph_TIM1



// TIM1 PB13 output
#define            PWM_2D_TIM_CH1N_GPIO_CLK      RCC_APB2Periph_GPIOB
#define            PWM_2D_TIM_CH1N_PORT          GPIOB
#define            PWM_2D_TIM_CH1N_PIN           GPIO_Pin_13

// TIM1 PB15 output
#define            PWM_3D_TIM_CH3N_GPIO_CLK      RCC_APB2Periph_GPIOB
#define            PWM_3D_TIM_CH3N_PORT          GPIOB
#define            PWM_3D_TIM_CH3N_PIN           GPIO_Pin_15

void ADVANCE_TIM_Init(void);
void ADVANCE_2D_PWM_Config(int duty_2d);
void ADVANCE_3D_PWM_Config(int duty_3d);
void ADVANCE_TIM_GPIO_Config(void);


#endif
