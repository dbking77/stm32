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
 * stm32_cpp: a C++ stm32 library
 * Driver for a standard 8N1 serial port.
 *
 * Usage:
 *
 *  Usart<USART1_BASE, 32> usart1;
 *  void USART1_IRQHandler(void)
 *  {
 *    usart1.irq();
 *  }
 *  in main():
 *    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // APB2 also has USART6
 *    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // ABP1 also has USART3, UART4/5
 *    usart1.init(115200);
 *    NVIC_SetPriority(USART1_IRQn, 1);
 *    NVIC_EnableIRQ(USART1_IRQn);
 */

#ifndef _STM32_CPP_USART_H_
#define	_STM32_CPP_USART_H_

#include "rcc.hpp"

#define USART_FLAG_TXE      ((uint16_t)0x0080)
#define USART_FLAG_TC       ((uint16_t)0x0040)
#define USART_FLAG_RXNE     ((uint16_t)0x0020)
#define USART_FLAG_IDLE     ((uint16_t)0x0010)
#define USART_FLAG_ORE      ((uint16_t)0x0008)
#define USART_FLAG_NE       ((uint16_t)0x0004)
#define USART_FLAG_FE       ((uint16_t)0x0002)
#define USART_FLAG_PE       ((uint16_t)0x0001)

/* Limitations:
 *  1. no support for oversampling by 8
 *  2. mode is always 8N1, no parity
 */

template <unsigned int SIZE>
struct UsartFifo
{  
  uint8_t head;
  uint8_t tail;
  uint16_t buffer[SIZE];

  UsartFifo() : head(0), tail(0) {}

  inline int16_t pop()
  {
    // when head == tail, there is nothing in buffer
    int16_t data = -1;
    if(head != tail){
      data = buffer[tail];
      tail = (tail+1)%SIZE;
    }
    return data;    
  }

  inline void push(uint16_t value)
  {
    buffer[head] = value; 
    head = (head+1)%SIZE;
  }
};


template <unsigned int USART_PTR, unsigned int SIZE>
class Usart
{
  uint16_t overrun_errors;
  UsartFifo<SIZE> rxbuf;
  UsartFifo<SIZE> txbuf;

public:
  static void init(uint32_t baud_rate)
  {
    USART_TypeDef* const USARTx = reinterpret_cast<USART_TypeDef*>(USART_PTR);

    /* setup for 1 stop bit */
    USARTx->CR2 &= ~((uint32_t)USART_CR2_STOP);
    
    /* setup 8N1, no parity */
    USARTx->CR1 &= ~((uint32_t) (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS));
    USARTx->CR1 |= USART_CR1_RE | USART_CR1_TE;

    /* set baud rate */
    //uint32_t apbclock = 0;
    if((USART_PTR == USART1_BASE) || (USART_PTR == USART6_BASE))
      USARTx->BRR = RccImpl::get_pclk2() / baud_rate;
    else
      USARTx->BRR = RccImpl::get_pclk1() / baud_rate;

    /* enable rx interrupt and go... */
    USARTx->CR1 |= USART_CR1_RXNEIE;
    USARTx->CR1 |= USART_CR1_UE;
  }

  void write(uint16_t data)
  {
    USART_TypeDef* const USARTx = reinterpret_cast<USART_TypeDef*>(USART_PTR);
    // push data to tx buffer and enable txe interrupt
    __disable_irq();
    txbuf.push(data);
    USARTx->CR1 |= USART_CR1_TXEIE;
    __enable_irq();
  }  

  int16_t read()
  {
    __disable_irq();
    int16_t data = rxbuf.pop();
    __enable_irq();
    return data;
  }

  void irq()
  {
    USART_TypeDef* const USARTx = reinterpret_cast<USART_TypeDef*>(USART_PTR);

    // if ORE (overrun error) bit gets set it will also trigger an RX interrupt.
    // to clear the ORE error, first the SR register and then DR to clear the ORE
    uint32_t sr = USARTx->SR;
    uint32_t cr1 = USARTx->CR1;
    if (sr & USART_SR_ORE)
    {
      ++overrun_errors;
    }

    if (sr & USART_SR_RXNE)
    {
      rxbuf.push( (reinterpret_cast<USART_TypeDef*>(USARTx)->DR & (uint16_t)0x01ff ) );
    }

    if ((sr & USART_SR_TXE) && (cr1 & USART_CR1_TXEIE))
    {      
      int16_t data = txbuf.pop();
      if (data == -1)
      {
        // no data left, turn off TXE interrupt
        USARTx->CR1 &= ~USART_CR1_TXEIE;
      }
      else 
      {
        USARTx->DR = data;
      }
    }
  }

};

/* Similar to above, except it adds an enable pin and functions to toggle it:
 *  Usart<USART1_BASE, 32, gpio_typedef> usart1;
 */
template<unsigned int USARTx, unsigned int SIZE, typename ENABLE>
class UsartWithEnable : public Usart<USARTx, SIZE>
{
public:
  void setTX()
  {
    ENABLE::high();
  }

  void setRX()
  {
    ENABLE::low();
  }
};

#endif /* _STM32_CPP_USART_H_ */
