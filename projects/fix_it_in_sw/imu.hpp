/*
 * Driver for ADXL345/ITG3200
 * by: Michael Ferguson 
 */

#ifndef IMU_HPP_GUARD_2304239048
#define IMU_HPP_GUARD_2304239048

#include "stm32f4xx.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_dma.h"

#include "gpio.hpp"

#define IMU_FLAG_TIMEOUT        ((uint32_t)0x80*6)

/* I2C Device Addresses */
#define DEVICE_GYRO             0xD6
#define DEVICE_ACCEL            0x32

/* L3GD20 Register Table */
#define ADDR_GYRO_WHO_AM_I      0x0F

#define ADDR_GYRO_CTRL_REG1     0x20
#define ADDR_GYRO_CTRL_REG2     0x21
#define ADDR_GYRO_CTRL_REG3     0x22
#define ADDR_GYRO_CTRL_REG4     0x23
#define ADDR_GYRO_CTRL_REG5     0x24
#define ADDR_GYRO_REFERENCE     0x25
#define ADDR_GYRO_TEMP_OUT      0x26
#define ADDR_GYRO_STATUS_REG    0x27

#define ADDR_GYRO_XOUT_L        0x28
#define ADDR_GYRO_XOUT_H        0x29
#define ADDR_GYRO_YOUT_L        0x2A
#define ADDR_GYRO_YOUT_H        0x2B
#define ADDR_GYRO_ZOUT_L        0x2C
#define ADDR_GYRO_ZOUT_H        0x2D

/* LSM303 Register Table */
#define ADDR_ACCEL_CTRL_REG1_A  0x20
#define ADDR_ACCEL_STAT_REG_A   0x27

#define ADDR_ACCEL_X_L_A        0x28
#define ADDR_ACCEL_X_H_A        0x29
#define ADDR_ACCEL_Y_L_A        0x2A
#define ADDR_ACCEL_Y_H_A        0x2B
#define ADDR_ACCEL_Z_L_A        0x2C
#define ADDR_ACCEL_Z_H_A        0x2D

#define I2C_SPEED               100000

// List of different I2C errors that could occur
enum imu_i2c_error_t {
  I2C_ERROR_START_BIT         = 1, // error occurred while sending i2c start
  I2C_ERROR_DEVICE_ADDR_WRITE = 2, // error occurred while sending i2c device address
  I2C_ERROR_REG_ADDR_WRITE    = 3, // error occurred while writing register address to device
  I2C_ERROR_DATA_WRITE        = 4, // error occurred while writing data to device
  I2C_ERROR_TIMEOUT           = 5, // general timeout error from mainloop of device (most likely occurring during DMA)
};

// List of IMU opperations that could lead to error 
enum imu_error_op_t {
  IMU_ERROR_WRITE_REG      = 1, // error occurred while writing byte to register
  IMU_ERROR_WRITE_ADDR     = 2, // error occurred while wrtting register address
  IMU_ERROR_READ_REG       = 3, // error occurred while reading register data
  IMU_ERROR_READ_DMA       = 4, // error occurred while reading register data DMA
};


enum imu_state_t {
  IMU_IDLE = 0,
  IMU_READING_GYRO = 1,           // waiting for gyro to return
  IMU_TIMING_GYRO = 2,            // waiting a bit, for timing considerations
  IMU_READING_ACCELEROMETER = 3,  // waiting for accelerometer to return
  IMU_TIMING_ACCELEROMETER = 4,   // waiting a bit, for timing considerations
}; 

struct imu_error_t
{
  imu_i2c_error_t i2c;   // I2C operation that was being performed during error
  imu_error_op_t  op;    // 
  imu_state_t     state;
  uint32_t        timeouts;
};



struct imu_data_t
{
  uint16_t accel_x;
  uint16_t accel_y;
  uint16_t accel_z;
  uint8_t  gyro_temp;
  uint8_t  gyro_sr;
  uint16_t gyro_x;
  uint16_t gyro_y;
  uint16_t gyro_z;
};


template<int _I2C_PTR, int _DMA_STREAM, int _DMA_CHANNEL, typename IMU_SCL, typename IMU_SDA>

class IMU
{

  imu_state_t imu_i2c_state; 
  uint32_t imu_i2c_timer;    // copy of system_time when current I2C transfer was started, used to detect timeouts
  imu_error_t imu_error;     // last error that occurred

  imu_data_t imu_data;   // more recent completely read IMU data
  imu_data_t imu_buffer; // buffer for storing IMU data while reading is in progress



 
  /* Handle a failed start */
  int32_t imu_timeout_callback(imu_i2c_error_t error_i2c, imu_error_op_t error_op)
  {
    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) _I2C_PTR;
    DMA_Stream_TypeDef* const DMAy_Streamx = (DMA_Stream_TypeDef*) _DMA_STREAM;

    I2C_GenerateSTOP(I2Cx, DISABLE);

    // disable what DMA RX or TX?
    DMA_Cmd(DMAy_Streamx, DISABLE);
    I2C_DMACmd(I2Cx,DISABLE);
    DMA_ClearFlag(DMAy_Streamx, DMA_FLAG_TCIF7);

    // track what caused error
    ++imu_error.timeouts;
    imu_error.i2c = error_i2c;
    imu_error.op = error_op;
    imu_error.state = imu_i2c_state;

    // pull SCL (PB6) low to release SDA
    IMU_SCL::mode(GPIO_OUTPUT_2MHz);
    IMU_SCL::low();
    delay_us(100);

    // restore SCL
    IMU_SCL::mode(GPIO_ALTERNATE_OD_2MHz | GPIO_AF_I2C2);

    // hacky hacky -- see https://kforge.ros.org/PRX/trac/ticket/37
    I2C_DeInit(I2Cx);
    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
    I2C_Init(I2Cx, &I2C_InitStructure);
    I2C_Cmd(I2Cx, ENABLE);

    return -1;    
  }

  /** Start a read of register <reg_addr> from device with I2C address <i2c_addr).
   *  Once read is started, data is transfered using DMA
   *
   *  The L3GD20 and LSM303DLHC both use a similar protocal over I2C.  Once I2C device is
   *  addressed a register address and auto-increment flag is sent to device. 
   *  Bits 6:0 of <reg_addr> is the slave register address, bit 7 is auto-increment flag
   * 
   *  Once a the register address is written, an second I2C transfer is started to 
   *  read data from device.
   */
  int32_t imu_start_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t num_bytes)
  {
    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) _I2C_PTR;
    DMA_Stream_TypeDef* const DMAy_Streamx = (DMA_Stream_TypeDef*) _DMA_STREAM;

    uint32_t dma_channel = (_DMA_CHANNEL & 7) << 25;


    // Config DMA to move data from IC2 peripheral to memory
    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = dma_channel;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(I2Cx->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = num_bytes;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_DeInit(DMAy_Streamx);
    DMA_Init(DMAy_Streamx, &DMA_InitStructure);

    // Enable DMA NACK
    I2C_DMALastTransferCmd(I2Cx, ENABLE);

    uint32_t timeout;
    
    // First transfer, write register address to device
    {
      // Generate I2C start bit
      I2C_GenerateSTART(I2Cx, ENABLE);

      // If I2C bus is busy, sending of start bit is delayed until it is idle.  
      // Wait for SB=1 to be set in status register
      timeout = IMU_FLAG_TIMEOUT;
      while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)){
        if((timeout--) == 0) return imu_timeout_callback(I2C_ERROR_START_BIT, IMU_ERROR_WRITE_ADDR);
      }

      // Send device I2C address, and start I2C write transmission
      I2C_Send7bitAddress(I2Cx, i2c_addr, I2C_Direction_Transmitter);

      // After I2C address is sent, then device should ACK its address,
      // and the uC I2C hardware state should tranistion to MASTER_TRANSMITTER_MODE 
      // Wait for status register bit ADDR=1
      timeout = IMU_FLAG_TIMEOUT;
      while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
        if((timeout--) == 0) return imu_timeout_callback(I2C_ERROR_DEVICE_ADDR_WRITE, IMU_ERROR_WRITE_ADDR);
      }

      // Send device the register address to read from
      I2C_SendData(I2Cx, reg_addr);  

      // Test on TXE FLag (data sent)
      timeout = IMU_FLAG_TIMEOUT;
      while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF))){
        if((timeout--) == 0) return imu_timeout_callback(I2C_ERROR_REG_ADDR_WRITE, IMU_ERROR_WRITE_ADDR);
      }
    }


    // Second I2C transfer : Start read of data from selected register. 
    {
      // Send START condition a second time  
      I2C_GenerateSTART(I2Cx, ENABLE);

      // Wait for SB=1
      timeout = IMU_FLAG_TIMEOUT;
      while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)){
        if((timeout--) == 0) return imu_timeout_callback(I2C_ERROR_START_BIT, IMU_ERROR_READ_REG);
      }
  
      // Send I2C address of device and start I2C read transfer
      I2C_Send7bitAddress(I2Cx, i2c_addr, I2C_Direction_Receiver);

      // After I2C address is send, then device should ACK its address
      // and the uC I2C hardware state should tranistion to MASTER_RECIEVER_MODE 
      timeout = IMU_FLAG_TIMEOUT;
      while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
        if((timeout--) == 0) return imu_timeout_callback(I2C_ERROR_DEVICE_ADDR_WRITE, IMU_ERROR_READ_REG);
      }
    }


    // Once read transmission has been started, let DMA take over
    {
      // Enable I2C DMA request
      I2C_DMACmd(I2Cx,ENABLE);
      // Enable DMA RX Channel
      DMA_Cmd(DMAy_Streamx, ENABLE);
    }
    
    // will process later!
    return 0;
  }


  /* DMA is done, shut things down */
  int32_t imu_finish_read(void)
  {
    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) _I2C_PTR;
    DMA_Stream_TypeDef* const DMAy_Streamx = (DMA_Stream_TypeDef*) _DMA_STREAM;

    // Send STOP
    I2C_GenerateSTOP(I2Cx, ENABLE);

    // Disable DMA RX Channel
    DMA_Cmd(DMAy_Streamx, DISABLE);

    // Disable I2C DMA request
    I2C_DMACmd(I2Cx,DISABLE);

    // Clear DMA RX Transfer Complete Flag
    uint32_t flag_tc;
    switch(_DMA_CHANNEL)
    {
    case 0 :flag_tc = DMA_FLAG_TCIF0;
    case 1 :flag_tc = DMA_FLAG_TCIF1;
    case 2 :flag_tc = DMA_FLAG_TCIF2;
    case 3 :flag_tc = DMA_FLAG_TCIF3;
    case 4 :flag_tc = DMA_FLAG_TCIF4;
    case 5 :flag_tc = DMA_FLAG_TCIF5;
    case 6 :flag_tc = DMA_FLAG_TCIF6;
    case 7 :flag_tc = DMA_FLAG_TCIF7;
    }
    DMA_ClearFlag(DMAy_Streamx, flag_tc);

    return 0;
  }


  /* Writes a single byte to register <reg_addr> of I2C device with addr <i2c_addr> */
  int32_t imu_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
  {
    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) _I2C_PTR;

    // Start
    I2C_GenerateSTART(I2Cx, ENABLE);

    uint32_t timeout;

    // Wait for SB=1
    timeout = IMU_FLAG_TIMEOUT;
    while(!I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB)){
      if((timeout--) == 0) return imu_timeout_callback(I2C_ERROR_START_BIT, IMU_ERROR_WRITE_REG);
    }

    // Send device address
    I2C_Send7bitAddress(I2Cx, i2c_addr, I2C_Direction_Transmitter);

    // Wait for ADDR=1
    timeout = IMU_FLAG_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
      if((timeout--) == 0) return imu_timeout_callback(I2C_ERROR_DEVICE_ADDR_WRITE, IMU_ERROR_WRITE_REG);
    }

    // Send register address
    I2C_SendData(I2Cx, reg_addr);  

    // Test on TXE FLag (data sent)
    timeout = IMU_FLAG_TIMEOUT;
    while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF))){
      if((timeout--) == 0) return imu_timeout_callback(I2C_ERROR_REG_ADDR_WRITE, IMU_ERROR_WRITE_REG);
    }
  
    // send data    
    I2C_SendData(I2Cx, data);

    // Test on TXE FLag (data sent)
    timeout = IMU_FLAG_TIMEOUT;
    while ((!I2C_GetFlagStatus(I2Cx,I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF))){
      if((timeout--) == 0) return imu_timeout_callback(I2C_ERROR_DATA_WRITE, IMU_ERROR_WRITE_REG);
    }

    // Send STOP
    I2C_GenerateSTOP(I2Cx, ENABLE);

    return 0;

  }


  /* Accelerometer does not measure at startup, need to configure it to do so */
  int32_t imu_start_accelerometer(void)
  {
    /* Enable Accelerometer via Control Register 1
     *  bits 7:4 = data rate = 0111b (Normal / low-power mode (400hz))
     *  bit  3   = power on = 0b  
     *  bits 2:0 = Z/Y/X on = 111b
     */
    imu_write(DEVICE_ACCEL, ADDR_ACCEL_CTRL_REG1_A, 0x77);

    return 0;
  }

  /* Gyro needs configuration at startup */
  int32_t imu_start_gyro(void)
  {
    /* Enable Gyro via Control Register 1
     *  bits 7:6 = data rate = 10b (380hz update rate)
     *  bits 5:4 = bandwidth = 11b (100hz cutoff)
     *  bit  3   = power on = 1b  
     *  bits 2:0 = Z/Y/X on = 111b
     */
    imu_write(DEVICE_GYRO, ADDR_GYRO_CTRL_REG1, 0xBF);

    /* Change Full Scale Selection
     *  bit  7   = block update (default: 0)
     *  bit  6   = lsb (0)
     *  bit  5:4 = full scale = 11b (2000dps)
     *                              (=70mdps/digit)
     */
    imu_write(DEVICE_GYRO, ADDR_GYRO_CTRL_REG4, 0x30);

    return 0;
  }


  // Determine the difference between two unsigned 32bit time values.
  //  returns t1-t2
  // Difference will be signed 32bit value.  
  // By using this function, it possible to handle wrap-around of timer value properly
  // as long as the difference between two time values is > -2147483648 and < 2147483647
  // if timeval is counting in milliseconds, the range is roughly +/-24days
  static int32_t timediff(uint32_t t1, uint32_t t2)
  {
    // for 32-bit values it is this simple
    return int32_t(t1-t2);
  }


public:

  /* Initialize the I2C and DMA */
  void init_imu(void)
  {
    I2C_TypeDef* const I2Cx = (I2C_TypeDef*) _I2C_PTR;

    imu_i2c_state = IMU_IDLE;


    // Enable I2C2 and DMA1
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Configure SCL and SDA pins as alternate function I2C pins

    // mode bits
    // bits 3:0 = AFR
    // bits 5:4 = MODER
    // bits 7:6 = OSPEEDR
    // bit  8   = OTYPER
    // GPIO_ALTERNATE_2MHz = 0x20 

    // MODER = 10
    // OTYPER = 1
    // OSPEEDR = ?
    // PUPDR = 00 
    // AFR = AF4 = 100
    // mode =  1 ?? 10 0100  = 1 0010 0100 = 0x124
    IMU_SCL::mode(GPIO_ALTERNATE_OD_2MHz | GPIO_AF_I2C2);
    IMU_SDA::mode(GPIO_ALTERNATE_OD_2MHz | GPIO_AF_I2C2);


    // I2C1
    I2C_DeInit(I2Cx);
    I2C_InitTypeDef I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;   // I2C (as opposed to SMBus)
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2Cx, &I2C_InitStructure);
    I2C_Cmd(I2Cx, ENABLE);
  }



  /* This is called from the main loop. It takes action based on our current state and whether the 
   *  DMA has finished reading yet. Also handles timeouts where data has not been returned. 
   *  
   * The system_time input should be counter that increaments every millisecond, it is used for detecting
   * timeouts of I2C transfers
   */
  void imu_update(uint32_t system_time)
  {
    DMA_Stream_TypeDef* const DMAy_Streamx = (DMA_Stream_TypeDef*) _DMA_STREAM;

    // function uses flag_tc
    uint32_t flag_tc;
    switch(_DMA_CHANNEL)
    {
    case 0 :flag_tc = DMA_FLAG_TCIF0;
    case 1 :flag_tc = DMA_FLAG_TCIF1;
    case 2 :flag_tc = DMA_FLAG_TCIF2;
    case 3 :flag_tc = DMA_FLAG_TCIF3;
    case 4 :flag_tc = DMA_FLAG_TCIF4;
    case 5 :flag_tc = DMA_FLAG_TCIF5;
    case 6 :flag_tc = DMA_FLAG_TCIF6;
    case 7 :flag_tc = DMA_FLAG_TCIF7;
    }


    if( imu_i2c_state == IMU_READING_ACCELEROMETER )
    {
      // out of time?
      if(timediff(system_time,imu_i2c_timer) > 2) 
      {
        imu_timeout_callback(I2C_ERROR_TIMEOUT, IMU_ERROR_READ_DMA);
        imu_i2c_timer = system_time;
        imu_i2c_state = IMU_TIMING_ACCELEROMETER;
        return;
      }

      // still receiving?
      if( !DMA_GetFlagStatus(DMAy_Streamx, flag_tc)) return;

      // Setup for next cycle
      imu_finish_read();
      imu_i2c_timer = system_time;
      imu_i2c_state = IMU_TIMING_ACCELEROMETER;
    }


    else if( imu_i2c_state == IMU_TIMING_ACCELEROMETER )
    {
      if(timediff(system_time,imu_i2c_timer) > 2)
      {
        imu_i2c_state = IMU_READING_GYRO;
        imu_start_read(DEVICE_GYRO, ADDR_GYRO_TEMP_OUT | 0x80, (uint8_t *) &imu_buffer.gyro_temp, 8);
        imu_i2c_timer = system_time;
      }
    }


    else if( imu_i2c_state == IMU_READING_GYRO )
    {
      // out of time?
      if(timediff(system_time,imu_i2c_timer) > 3)
      {
        imu_timeout_callback(I2C_ERROR_TIMEOUT, IMU_ERROR_READ_DMA);
        imu_i2c_timer = system_time;
        imu_i2c_state = IMU_TIMING_GYRO;
        return;
      }
      // still receiving?
      if( !DMA_GetFlagStatus(DMAy_Streamx, flag_tc) ) return;

      // Setup for next cycle
      imu_finish_read();
      imu_i2c_timer = system_time;
      imu_i2c_state = IMU_TIMING_GYRO;
    }
  

    else if( imu_i2c_state == IMU_TIMING_GYRO )
    {
      // accellerometer and gyro transfer is complete, copy over buffered data
      imu_data = imu_buffer;

      if(timediff(system_time,imu_i2c_timer) > 2)
      {
        imu_i2c_state = IMU_READING_ACCELEROMETER;
        imu_start_read(DEVICE_ACCEL, ADDR_ACCEL_X_L_A | 0x80, (uint8_t *) &imu_buffer.accel_x, 6);
        imu_i2c_timer = system_time;
      }
    }

  
    else
    {
      imu_start_accelerometer();
      imu_start_gyro();
      imu_i2c_state = IMU_READING_GYRO;
      imu_start_read(DEVICE_GYRO, ADDR_GYRO_TEMP_OUT | 0x80, (uint8_t *) &imu_buffer.gyro_temp, 8);
      imu_i2c_timer = system_time;
    }    
  }

};

#endif //IMU_HPP_GUARD_2304239048

