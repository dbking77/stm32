#include "capture_channel.hpp"
#include "stm32f4xx.h"
#include "gpio.hpp"
#include "stm32f4xx_tim.h"


// Configures signal timer channel to support two capture inputs on CH1 and CH2
// currently hard-coded to support sonar (50ms repeat rate)
template<int _TIMER16_PTR, typename CH1_GPIO, typename CH2_GPIO>
class DualPulseTimer
{

  CaptureChannel ch1, ch2;

  TIM_TypeDef* tim;

  // sonar pulse width output : 147uSec/inch 
  // max range 254 inches ~= 37msec
  // repeat rate ~20Hz = 50ms

public:


  // returns pulse width in uSec, returns 0 if no pulses have been seen for last ~200mSec
  uint32_t getCh1PulseWidthUsec()
  {
    __disable_irq();
    uint32_t width = ch1.getPulseWidth();
    __enable_irq();
    // 1/Fclk * prescaler * 1e6usec/sec 
    // 1/168e6 * 128 * 1e6 = 0.7619 usec per tick
    // 195/256 = 0.76171875 ~= 7619
    return (width * 195) >> 8; 
  }

  // returns pulse width in uSec, returns 0 if no pulses have been seen for last ~200mSec
  uint32_t getCh2PulseWidthUsec()
  {
    __disable_irq();
    uint32_t width = ch2.getPulseWidth();
    __enable_irq();
    // 1/Fclk * prescaler * 1e6usec/sec 
    // 1/168e6 * 128 * 1e6 = 0.7619 usec per tick
    // 195/256 = 0.76171875 ~= 7619
    return (width * 195) >> 8; 
  }



  void init()
  {
    TIM_TypeDef* const TIMx = (TIM_TypeDef* const)  _TIMER16_PTR;
    tim = TIMx;

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

    // clear any interrupt flags
    TIM_ClearFlag(TIMx, TIM_FLAG_CC1OF);
    TIM_ClearFlag(TIMx, TIM_FLAG_CC2OF);
    TIM_ClearFlag(TIMx, TIM_FLAG_CC1);
    TIM_ClearFlag(TIMx, TIM_FLAG_CC2);
    TIM_ClearFlag(TIMx, TIM_FLAG_Update);

    // enable interrupt on input capture and update (overflow) events
    TIM_ITConfig(TIMx, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(TIMx, TIM_IT_CC2, ENABLE);
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIMx, ENABLE);
  }

  void irq()
  {
    TIM_TypeDef* const TIMx = (TIM_TypeDef* const)  _TIMER16_PTR;
    
    if (TIM_GetFlagStatus(TIMx, TIM_FLAG_CC1))
    {
      // use pin state to determine if event was a rising or falling edge
      CaptureChannel::Edge edge = (CH1_GPIO::value()) ? CaptureChannel::CC_RISING_EDGE : CaptureChannel::CC_FALLING_EDGE;
      // get capture count
      uint32_t cnt = TIM_GetCapture1(TIMx);
      
      // see if there was an overcapture (missed capture event)
      if (TIM_GetFlagStatus(TIMx, TIM_FLAG_CC1OF))
      {
        // there was over=capture.  assume all data is garbage and start again
        TIM_ClearFlag(TIMx, TIM_FLAG_CC1OF);
        ch1.error();
      }
      else 
      {
        // there was no capture event
        ch1.event(cnt, edge);
      }

      //TIM_ClearFlag(TIMx, TIM_FLAG_CC1);      
    }

    if (TIM_GetFlagStatus(TIMx, TIM_FLAG_CC2))
    {
      // use pin state to determine if event was a rising or falling edge
      CaptureChannel::Edge edge = (CH2_GPIO::value()) ? CaptureChannel::CC_RISING_EDGE : CaptureChannel::CC_FALLING_EDGE;
      // get capture count
      uint32_t cnt = TIM_GetCapture2(TIMx);
      
      // see if there was an overcapture (missed capture event)
      if (TIM_GetFlagStatus(TIMx, TIM_FLAG_CC2OF))
      {
        // there was over=capture.  assume all data is garbage and start again
        TIM_ClearFlag(TIMx, TIM_FLAG_CC2OF);
        ch2.error();
      }
      else 
      {
        // there was no capture event
        ch2.event(cnt, edge);
      }

      //TIM_ClearFlag(TIMx, TIM_FLAG_CC1);      
    }

        
    if (TIM_GetFlagStatus(TIMx, TIM_FLAG_Update))
    {
      TIM_ClearFlag(TIMx, TIM_FLAG_Update);
      ch1.overflow();
      ch2.overflow();
    }



  } //end irq()


};
