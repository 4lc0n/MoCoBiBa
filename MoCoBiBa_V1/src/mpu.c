#include "mpu.h"

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// register in order to set to cycle and interrupt on accel_z // 105 was 57 (dunno why)
const uint8_t lowpower_regs[8]  = {107, 108, 29,   56, 105,  31, 30, 107};      // only axis x activated, wake on motion activated, sampling at  4 Hz
const uint8_t lowpower_data[8] = {   1,0x37,  5, 0x40,0xC0,0x04,  4, 0x29};     // threshhold at reg 31

// registers in order to set to sampling at 125 Hz, all 3 accel axes
const uint8_t sampling_regs[9] = {107, 108, 28, 29,  56, 105,   31,  30, 107};   // wake on motion activated, sample-rate: 125, +-2g, gyro deactivated
const uint8_t sampling_data[9] = {  1,0x07,  1,  0,0x40,0xC0, 0x04,0x08, 0x09};  //  

const uint8_t sleep_regs[3] = {108, 107, 107};
const uint8_t sleep_data[3] = {0x3F, 0x08, 0x40};



bool mpu_set_register(uint8_t reg, uint8_t data)
{

  if(PRR & (0x01 << PRUSI))
  {
    // reactivate the USI interface:
    PRR &= ~(0x01 << PRUSI);
    _delay_ms(1);
  }
  uint8_t msg[3] = {mpu_address | I2C_WRITE, reg, data};

  if(!usiTwiStartTransceiverWithData(msg, 3))
  {
       //failed to transmit
    return false;
  }
  
  return true;

}


bool mpu_set_registers(const uint8_t *reg, const uint8_t *data, uint8_t number)
{
  if(PRR & (0x01 << PRUSI))
  {
    // reactivate the USI interface:
    PRR &= ~(0x01 << PRUSI);
    _delay_ms(1);
  }
  uint8_t i = 0;
  for(i = 0; i < number; i++)
  {
    if(!mpu_set_register(reg[i], data[i]))
    {
      return false;
    }
    _delay_us(10);
  }
  return true;
}

bool mpu_get_registers(uint8_t reg, uint8_t *data, uint8_t number)
{
  if(PRR & (0x01 << PRUSI))
  {
    // reactivate the USI interface:
    PRR &= ~(0x01 << PRUSI);
    _delay_ms(1);
  }

  uint8_t msg[number + 1];
  msg[0] = mpu_address | I2C_WRITE;
  msg[1] = reg;

  //transmit reg to read from
  if(usiTwiStartTransceiverWithData(msg, 2))
  {
    _delay_us(50);
  }
  else{     //failed to transmit
    return false;
  }

  msg[0] = mpu_address | I2C_READ;
  // read from MPU
  if(usiTwiStartTransceiverWithData(msg, number + 1))
  {
    uint8_t i;
    for(i = 0; i < number; i++)
    {
      data[i] = msg[i + 1];
    }

    return true;
  }
  else
  {
    return false;
  }

  return true;
}


bool mpu_get_register(uint8_t reg, uint8_t* data)
{
  if(PRR & (0x01 << PRUSI))
  {
    // reactivate the USI interface:
    PRR &= ~(0x01 << PRUSI);
    _delay_ms(1);
  }
  
  uint8_t msg[2] = {mpu_address | I2C_WRITE, reg};

  if(usiTwiStartTransceiverWithData(msg, 2))
  {
    _delay_us(50);
  }
  else{     //failed to transmit
    return false;
  }

  msg[0] = mpu_address | I2C_READ;
  

  if(usiTwiStartTransceiverWithData(msg, 2))
  {
    *data = msg[1];
    return true;
  }
  else
  {
    *data = 0;
    return false;
  }
}



bool mpu_set_interrupt()
{
  
  //reset MPU unit
  mpu_set_register(107, 0x80);
  _delay_ms(100);
  
  
  if(mpu_set_registers(lowpower_regs, lowpower_data, sizeof(lowpower_regs)))
  {
    return true;
  }
  else
  {
    return false;
  }
  
  
}



bool mpu_set_sampling()
{
  //reset MPU unit
  mpu_set_register(107, 0x80);
  _delay_ms(100);

  // for(uint8_t i = 0; i < 9; i++)
  // {
  //   if(!mpu_set_register(sampling_regs[i], sampling_data[i]))
  //   {
  //     //failed to transmit
  //     return false;
  //   }
  //   else{
  //     //wait for a short period of time to distinquish the data packets on the logic analyzer
  //     _delay_us(100);
  //   }
  // }

  if(!mpu_set_registers(sampling_regs, sampling_data, sizeof(sampling_regs)))
  {
    return false;
  }
  return true;

  //everything seemed to work
}

bool mpu_set_sleep()
{
    mpu_set_register(107, 0x80);
  _delay_ms(100);

  // sets the MPU in sleep mode, deactivates all sensors
  if(!mpu_set_registers(sleep_regs, sleep_data, sizeof(sleep_regs)))
  {
    return false;
  }
  else{
    return true;
  }
}


//read data from regs 59 to 72 and pass back a struct
//using the fast way, read 14 registers at once
//doesn't use the mpu_get_register function
bool mpu_read_regs(struct mpu_data *ret_data)
{
  if(PRR & (0x01 << PRUSI))
  {
    // reactivate the USI interface:
    PRR &= ~(0x01 << PRUSI);
    _delay_ms(1);
  }

  uint8_t data[15];
  data[0] = mpu_address | 0;
  data[1] = 59;
  //indicate at which address to start reading
  if(usiTwiStartTransceiverWithData(data, 2))
  {
    return false;
  }

  data[0] = mpu_address | I2C_READ;

  //read 14 regs at once
  if(!usiTwiStartTransceiverWithData(data, 15))
  {
    return false;
  }


  ret_data->accel_x = data[1] << 8 | data[2];
  ret_data->accel_y = data[3] << 8 | data[4];
  ret_data->accel_z = data[5] << 8 | data[6];

  ret_data->temp = data[7] << 8 | data[8];

  ret_data->gyro_x = data[9] << 8 | data[10];
  ret_data->gyro_y = data[11] << 8 | data[12];
  ret_data->gyro_z = data[13] << 8 | data[14];

  return true;

}

bool mpu_get_wai()
{
  uint8_t temp = 0;
  if(!mpu_get_register(117, &temp))
  {
    return false;
  }
  if(temp != MPU_WAI)
  {
    return false;
  }
  else
  {
    return true;
  }
  
}