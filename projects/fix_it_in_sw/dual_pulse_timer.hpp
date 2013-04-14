#include "stm32f4xx.h"
#include "gpio.hpp"
#include "stm32f4xx_tim.h"


// acts like a state machine for a single capture channel,
// it keeps track of what next event should be, last event time, and errors
class CaptureChannel
{
  
  uint32_t last_falling_edge;
  uint32_t last_rising_edge;
  
  enum State {CC_STATE_INIT=0, CC_STATE_WAITING_FOR_FALLING=1, CC_STATE_WAITING_FOR_RISING=2} state;
  uint32_t pulse_width;
  uint32_t pulse_count; // count of valid pulses
  
  uint32_t pulse_period; // more for debugging 

  uint32_t errors;
  uint32_t updates; 

public:
  enum Edge {CC_RISING_EDGE=1, CC_FALLING_EDGE=0};


  void event(uint32_t cnt, Edge edge)
  {
    switch (state)
    {
    case CC_STATE_INIT:
      if (edge == CC_RISING_EDGE)
      {
        state = CC_STATE_WAITING_FOR_FALLING;
      }
      // else do nothing, still waiting for first rising edge
      break;
    case CC_STATE_WAITING_FOR_FALLING:
      if (edge == CC_FALLING_EDGE)
      {
        state = CC_STATE_WAITING_FOR_RISING;
        pulse_width = (cnt - last_rising_edge) & 0xFFFF;
        ++pulse_count;
      }
      else
      {
        // unexpected edge 
        error();
      }
      break;
    case CC_STATE_WAITING_FOR_RISING:
      if (edge == CC_RISING_EDGE)
      {
        state = CC_STATE_WAITING_FOR_FALLING;
        pulse_period = (cnt - last_rising_edge) & 0xFFFF;
      }
      else
      {
        error();
      }
      break;
    default:
      error();
    }

    if (edge == CC_RISING_EDGE)
    {
      last_rising_edge = cnt;
    }
    else
    {
      last_falling_edge = cnt;
    }    
    ++updates;
  }

  void error()
  {
    state = CC_STATE_INIT;
    ++errors;    
  }

};




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

    // enable interrupt on input caputure events
    TIM_ITConfig(TIMx, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(TIMx, TIM_IT_CC2, ENABLE);

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

  } //end irq()


};
