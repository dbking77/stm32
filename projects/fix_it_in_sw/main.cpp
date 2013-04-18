/*
 * Copyright (c) 2012, Michael E. Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * PROJECT: Fix-it-in-software
 * Firmware for custom F4-based battlebot controller.
 * author: Michael E. Ferguson
 */

#include "stm32f4xx.h"

#include "gpio.hpp"
#include "usart.hpp"
#include "encoder.hpp"
#include "ncv7729.hpp"
#include "imu.hpp"
#include "pulse_timer.hpp"
#include "dual_pulse_timer.hpp"
#include "servo_out.hpp"

extern "C"
{
#include "comm.h"
}

typedef Gpio<GPIOA_BASE,0> left_enc_a;
typedef Gpio<GPIOA_BASE,1> left_enc_b;
typedef Gpio<GPIOA_BASE,2> radio_gpio1; /* TIM9, CH1 */
typedef Gpio<GPIOA_BASE,3> left_CS;
typedef Gpio<GPIOA_BASE,5> radio_gpio2; /* TIM2, CH1 */
typedef Gpio<GPIOA_BASE,7> tim1_ch1n;
typedef Gpio<GPIOA_BASE,8> tim1_ch1;
typedef Gpio<GPIOA_BASE,9> tim1_ch2;

typedef Gpio<GPIOB_BASE,0> tim1_ch2n;
typedef Gpio<GPIOB_BASE,1> current_sense; /* A9 */
typedef Gpio<GPIOB_BASE,5> motor_enable;
typedef Gpio<GPIOB_BASE,6> serial_tx; /* USART1 */
typedef Gpio<GPIOB_BASE,7> serial_rx;
typedef Gpio<GPIOB_BASE,8> radio_gpio3; /* TIM10, CH1 */
typedef Gpio<GPIOB_BASE,9> radio_gpio4; /* TIM11, CH1 */
typedef Gpio<GPIOB_BASE,10> imu_scl; /* I2C2 */
typedef Gpio<GPIOB_BASE,11> imu_sda;
typedef Gpio<GPIOB_BASE,13> sck; /* SPI2 */
typedef Gpio<GPIOB_BASE,14> miso;
typedef Gpio<GPIOB_BASE,15> mosi;

typedef Gpio<GPIOC_BASE,1> voltage_sense; /* A11 */
typedef Gpio<GPIOC_BASE,5> left_fault;
typedef Gpio<GPIOC_BASE,6> right_enc_a;
typedef Gpio<GPIOC_BASE,7> right_enc_b;
typedef Gpio<GPIOC_BASE,8> right_CS;
typedef Gpio<GPIOC_BASE,9> right_fault;
typedef Gpio<GPIOC_BASE,10> top_r_led;
typedef Gpio<GPIOC_BASE,11> top_b_led;  // pins for green and blue are flipped 
typedef Gpio<GPIOC_BASE,12> top_g_led;
typedef Gpio<GPIOC_BASE,13> bot_r_led;
typedef Gpio<GPIOC_BASE,14> bot_b_led;
typedef Gpio<GPIOC_BASE,15> bot_g_led;


/* two attached encoders */
//Encoder<TIM5_BASE> left_enc;
//Encoder<TIM3_BASE> right_enc;
/* Use encoder inputs as pulse timers for sonar instead */
// Left Encoder TIM5 CH1 and CH2
// Right Encoder TIM3 CH1 and CH2
DualPulseTimer<TIM5_BASE, left_enc_a,  left_enc_b > left_sonars;
DualPulseTimer<TIM3_BASE, right_enc_a, right_enc_b> right_sonars;

// radio input channels
//PulseTimer<TIM9_BASE,  radio_gpio1> radio_ch1;
ServoOut<TIM9_BASE> servo_out1;
PulseTimer<TIM2_BASE,  radio_gpio2> radio_ch2;
PulseTimer<TIM10_BASE, radio_gpio3> radio_ch3;
PulseTimer<TIM11_BASE, radio_gpio4> radio_ch4;


IMU<I2C2_BASE, DMA1_Stream3_BASE, 3 /* DMA STREAM*/, 7 /* DMA_CHANNEL */, imu_scl, imu_sda> imu;

/* two NCV7729 motor drivers */
Ncv7729<SPI2_BASE, left_CS, TIM1_BASE, motor_enable, left_fault, 1> left_motor;
Ncv7729<SPI2_BASE, right_CS, TIM1_BASE, motor_enable, right_fault, 2> right_motor;

/* debugging via FTDI */
Usart<USART1_BASE, 32> usart1;

/* system clock */
volatile uint32_t system_clock;
volatile bool main_loop_update_flag;

volatile float v_motor_out2;
volatile float v_motor_out3;
volatile float v_motor_out4;
volatile float v_motor_out5;

volatile int16_t left_status, right_status;


void print(char const * str)
{
  while (*str!='\0')
  {
    usart1.write(*str);
    ++str;
  }
}

void print_gyro()
{
  print("gyro :");
  print(" x="); 
  print(dec2str(imu.gyro_data.x));
  print(" y=");
  print(dec2str(imu.gyro_data.y));
  print(" z=");
  print(dec2str(imu.gyro_data.z));
  print("\r\n");
}

void print_accel()
{
  print("accel :");
  print(" x="); 
  print(dec2str(imu.accel_data.x));
  print(" y=");
  print(dec2str(imu.accel_data.y));
  print(" z=");
  print(dec2str(imu.accel_data.z));
  print("\r\n");
}

volatile uint32_t pclk1_freq;
volatile uint32_t pclk2_freq;
volatile uint32_t sysclk_freq;
volatile uint32_t hclk_freq;

RCC_TypeDef *_rcc;

int main(void)
{

  pclk1_freq = RccImpl::get_pclk1();
  pclk2_freq = RccImpl::get_pclk2();
  sysclk_freq = RccImpl::get_sysclk();
  hclk_freq = RccImpl::get_hclk();

  _rcc = RCC;
  
  NVIC_SetPriorityGrouping(3);

  // enable all GPIO clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  top_r_led::mode(GPIO_OUTPUT);
  top_g_led::mode(GPIO_OUTPUT);
  top_b_led::mode(GPIO_OUTPUT);
  bot_r_led::mode(GPIO_OUTPUT);
  bot_g_led::mode(GPIO_OUTPUT);
  bot_b_led::mode(GPIO_OUTPUT);

  top_r_led::high();
  top_g_led::high();
  top_b_led::low();
  bot_r_led::high();
  bot_g_led::high();
  bot_b_led::high();

  // setup encoders
  /*
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM5EN;
  left_enc_a::mode(GPIO_ALTERNATE | GPIO_AF_TIM5);
  left_enc_b::mode(GPIO_ALTERNATE | GPIO_AF_TIM5);
  right_enc_a::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  right_enc_b::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  left_enc.init();
  right_enc.init();
  */

  // setup sonars
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM5EN;
  left_enc_a::mode(GPIO_ALTERNATE | GPIO_AF_TIM5);
  left_enc_b::mode(GPIO_ALTERNATE | GPIO_AF_TIM5);
  right_enc_a::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  right_enc_b::mode(GPIO_ALTERNATE | GPIO_AF_TIM3);
  left_sonars.init();
  right_sonars.init();
  NVIC_SetPriority(TIM5_IRQn, 1);
  NVIC_SetPriority(TIM3_IRQn, 1);
  NVIC_EnableIRQ(TIM5_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);

  // setup radio channel
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN;
  radio_gpio1::mode(GPIO_ALTERNATE | GPIO_AF_TIM9);
  radio_gpio2::mode(GPIO_ALTERNATE | GPIO_AF_TIM2);
  radio_gpio3::mode(GPIO_ALTERNATE | GPIO_AF_TIM10);
  radio_gpio4::mode(GPIO_ALTERNATE | GPIO_AF_TIM11);
  //radio_ch1.init();
  radio_ch2.init();
  radio_ch3.init();
  radio_ch4.init();
  //NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 1);
  NVIC_SetPriority(TIM2_IRQn, 1);
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1);
  NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 1);
  //NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

  servo_out1.init();
  


  // setup motors
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  tim1_ch1::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
  tim1_ch1n::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
  tim1_ch2::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
  tim1_ch2n::mode(GPIO_ALTERNATE | GPIO_AF_TIM1);
  sck::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
  miso::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
  mosi::mode(GPIO_ALTERNATE | GPIO_AF_SPI2);
  left_motor.init();
  right_motor.init();

  // setup usart
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  serial_tx::mode(GPIO_ALTERNATE_2MHz | GPIO_AF_USART1);
  serial_rx::mode(GPIO_ALTERNATE_2MHz | GPIO_AF_USART1);
  usart1.init(38400);
  NVIC_SetPriority(USART1_IRQn, 2);
  NVIC_EnableIRQ(USART1_IRQn);

  // setup systick
  imu.init_imu();

  // setup systick
  SysTick_Config(SystemCoreClock/1000);
  NVIC_EnableIRQ(SysTick_IRQn);
  system_clock = 0;
  __enable_irq();

  
  while(1)
  {
    if (main_loop_update_flag)
    {
      main_loop_update_flag = false;
      
      print("s "); print(dec2str(imu.accel_data.z));
      print(" "); print(dec2str(imu.gyro_data.z));
      //print(" "); print(dec2str(radio_ch1.getPulseWidthUsec()));
      print(" "); print(dec2str(radio_ch2.getPulseWidthUsec()));
      print("\r\n");

      v_motor_out2 = radio_ch2.getPulseWidthUsec();
      v_motor_out3 = radio_ch3.getPulseWidthUsec();
      v_motor_out4 = radio_ch4.getPulseWidthUsec();
      v_motor_out5 = right_sonars.getCh1PulseWidthUsec();

      int width = radio_ch2.getPulseWidthUsec();
      if (width == 0)
      {
        left_motor.set(0.0);
        right_motor.set(0.0);
      }
      else 
      {
        float motor_out = float(width); //-1500)* 0.002f;
        //v_motor_out = motor_out;
        //left_motor.set(motor_out);
        //right_motor.set(motor_out);
      }      

      /*
      float servo_out;
      if (width == 0)
      {
        servo_out = 0.0;
      }
      else 
      {
        servo_out = float(width-1000)* 0.001f;
      }
      servo_out1.setServoOutput(servo_out);
      */


    }

    if ((system_clock % 1000) == 0)
    {
      left_status = left_motor.read(NCV7729_RD_DIAG);
      right_status = right_motor.read(NCV7729_RD_DIAG);
      //print_gyro();
    }
    imu.imu_update(system_clock);
  }

}

extern "C"
{

void SysTick_Handler(void)
{
  system_clock++;
  if( system_clock % 1000 == 0 )
  {
    if( (system_clock / 1000) % 3 == 0)
    {
      //left_motor.set(0);
      //right_motor.set(0);
      top_b_led::high();
      top_r_led::low();
    }
    else if( (system_clock / 1000) % 3 == 1)
    {
      //left_motor.set(0.1);
      //right_motor.set(0.1);
      top_r_led::high();
      top_g_led::low();
    }
    else
    {
      //left_motor.set(-0.1);
      //right_motor.set(-0.1);
      top_g_led::high();
      top_b_led::low(); 
    }
  }
  if ((system_clock % 100) == 0)
  {
    main_loop_update_flag = true;
  }    
}

void USART1_IRQHandler(void)
{
  usart1.irq();
}

void TIM3_IRQHandler(void)
{
  right_sonars.irq();
}

void TIM5_IRQHandler(void)
{
  left_sonars.irq();
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
  //radio_ch1.irq();
}

void TIM2_IRQHandler(void)
{
  radio_ch2.irq();
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  radio_ch3.irq();
}

void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  radio_ch4.irq();
}


} /* extern C */
