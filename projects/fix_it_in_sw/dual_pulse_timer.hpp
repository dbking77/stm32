#include "stm32f4xx.h"
#include "gpio.hpp"
#include "stm32f4xx_tim.h"


template<int _TIMER16_PTR, typename CH1_IC_GPIO, typename CH2_IC_GPIO>
class DualPulseTimer
{

  // sonar pulse width output : 147uSec/inch 
  // max range 254 inches ~= 37msec
  // repeat rate ~20Hz = 50ms

public:
  void init()
  {
    TIM_TypeDef* const TIMx = (TIM_TypeDef* const)  _TIMER16_PTR;


    // Counter should have max resolution 0xFFFF, and prescaler should 
    // be set to give about 50ms period
    // 1/Fclk * 0xFFFF * prescaler = 50mSec
    // 1/160Mhz * 0xFFFF * 122 != 50mSecc
    
    TIM_TimeBaseInitTypeDef time_base_init;
    TIM_TimeBaseStructInit(&time_base_init);
    time_base_init.TIM_Prescaler = 120;  
    time_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    time_base_init.TIM_Period = 0xFFFF;
    time_base_init.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base_init.TIM_RepetitionCounter = 0;
    TIM_DeInit(TIMx);
    TIM_TimeBaseInit(TIMx, &time_base_init);

    // setup first input capture channel
    TIM_ICInitTypeDef time_ic_init;
    TIM_ICStructInit(&time_ic_init);
    time_ic_init.TIM_Channel = TIM_Channel_1;
    time_ic_init.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    time_ic_init.TIM_ICSelection = TIM_ICSelection_DirectTI;
    time_ic_init.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    time_ic_init.TIM_ICFilter = 0xF;  // use most agressive filter for now
    TIM_ICInit(TIMx, &time_ic_init);

    // setup second input capture channel
    TIM_ICStructInit(&time_ic_init);
    time_ic_init.TIM_Channel = TIM_Channel_2;
    time_ic_init.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    time_ic_init.TIM_ICSelection = TIM_ICSelection_DirectTI;
    time_ic_init.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    time_ic_init.TIM_ICFilter = 0xF;  // use most agressive filter for now
    TIM_ICInit(TIMx, &time_ic_init);
  }

};
