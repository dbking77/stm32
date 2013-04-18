#ifndef SERVO_OUT_HPP_GUARD_2348923749823
#define SERVO_OUT_HPP_GUARD_2348923749823


// Configures signal timer channel to output servo pulses on CH1
template<int _TIMER16_PTR>
class ServoOut
{

  TIM_TypeDef* tim;

  // sonar pulse width output : 147uSec/inch 
  // max range 254 inches ~= 37msec
  // repeat rate ~20Hz = 50ms
  
public:

  // output should be in range of 0.0 (for 1ms pulse) and 1.0 for 2.0 ms pulse
  void setServoOutput(float value)
  {    
    // In 1milisecond there are:
    //   Fclk/Prescaler*1ms = 168e6 / 51 * 1e-3 = 3294 ticks
    TIM_TypeDef* const TIMx = (TIM_TypeDef* const)  _TIMER16_PTR;
    int v = value * 3294 + 3294;
    if (v < 3294) v = 3294;
    if (v > 6588) v = 6588;
    TIM_SetCompare1(TIMx, v);
  }

  void init()
  {
    TIM_TypeDef* const TIMx = (TIM_TypeDef* const)  _TIMER16_PTR;
    tim = TIMx;

    // Counter should have max resolution 0xFFFF, and prescaler should 
    // be set to give about 20ms period
    // 1/Fclk * 0xFFFF * prescaler = period
    // prescaler = Fclk * period / 0xFFFF = 168e6 * 50e-3 / 0xFFFF ~= 51
    //
    // resolution of counter = prescaler / Fclk = 51 / 168e6 = 0.30357us

    TIM_TimeBaseInitTypeDef time_base_init;
    TIM_TimeBaseStructInit(&time_base_init);
    time_base_init.TIM_Prescaler = 51;    // 
    time_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    time_base_init.TIM_Period = 0xFFFF;
    time_base_init.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base_init.TIM_RepetitionCounter = 0;
    TIM_DeInit(TIMx);
    TIM_TimeBaseInit(TIMx, &time_base_init);

    TIM_OCInitTypeDef tim_oc_init;
    TIM_OCStructInit(&tim_oc_init);
    tim_oc_init.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc_init.TIM_Pulse = 3294;
    tim_oc_init.TIM_OCPolarity = TIM_OCNPolarity_High;
    tim_oc_init.TIM_OCIdleState = TIM_OSSIState_Disable;
    TIM_OC1Init(TIMx, &tim_oc_init);

    TIM_Cmd(TIMx, ENABLE);
  }


};



#endif // SERVO_OUT_HPP_GUARD_2348923749823
