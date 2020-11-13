#include <Arduino.h>
#include <usiTwiMaster.h>


#define I2C_READ 1
#define I2C_WRITE 0

struct mpu_data{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};




const uint8_t mpu_address = 0x68 << 1;


// register in order to set to cycle and interrupt on accel_x
const uint8_t lp_regs[8]  = {107, 108, 29, 56, 57, 31, 30, 107};
const uint8_t lp_data[8] = {1, 31, 1, 64, 192, 16, 4, 0x29};

const uint8_t sp_regs[9] = {107, 108, 28, 29, 56, 57, 31, 30, 107};
const uint8_t sp_data[9] = {1, 31, 1, 0,  64, 192, 16, 8, 0x9};


bool mpu_set_register(uint8_t reg, uint8_t data);
bool mpu_get_register(uint8_t reg, uint8_t* data);
bool mpu_set_registers(const uint8_t *reg, const uint8_t *data, uint8_t number);
bool mpu_get_registers(uint8_t reg, uint8_t *data, uint8_t number);
bool mpu_set_interrupt();
bool mpu_set_sampling();
bool mpu_read_regs(struct mpu_data *ret_data);


void setup() {
  // put your setup code here, to run once:

  pinMode(A1, OUTPUT);
  digitalWrite(A1, HIGH);
  usiTwiMasterInitialize();

  digitalWrite(A1, LOW);

  if(mpu_set_interrupt())
  {
    digitalWrite(A1, HIGH);

  }
  else
  {
    digitalWrite(A1, LOW);
  }
}

void loop() {
  
}





bool mpu_set_register(uint8_t reg, uint8_t data)
{
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
  uint8_t i = 0;
  for(i = 0; i < number; i++)
  {
    if(!mpu_set_register(reg[i], data[i]))
    {
      return false;
    }
    delayMicroseconds(10);
  }
  return true;
}

bool mpu_get_registers(uint8_t reg, uint8_t *data, uint8_t number)
{

  uint8_t msg[number + 1];
  msg[0] = mpu_address | I2C_WRITE;
  msg[1] = reg;

  //transmit reg to read from
  if(usiTwiStartTransceiverWithData(msg, 2))
  {
    delayMicroseconds(50);
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
    uint8_t msg[2] = {mpu_address | I2C_WRITE, reg};

  if(usiTwiStartTransceiverWithData(msg, 2))
  {
    delayMicroseconds(50);
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
  delay(100);
  
  
  if(mpu_set_registers(lp_regs, lp_data, sizeof(lp_regs)))
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
  delay(100);

  // for(uint8_t i = 0; i < 9; i++)
  // {
  //   if(!mpu_set_register(sp_regs[i], sp_data[i]))
  //   {
  //     //failed to transmit
  //     return false;
  //   }
  //   else{
  //     //wait for a short period of time to distinquish the data packets on the logic analyzer
  //     delayMicroseconds(100);
  //   }
  // }

  if(!mpu_set_registers(sp_regs, sp_data, sizeof(sp_regs)))
  {
    return false;
  }
  return true;

  //everything seemed to work
  return true;
}






//read data from regs 59 to 72 and pass back a struct
//using the fast way, read 14 registers at once
//doesn't use the mpu_get_register function
bool mpu_read_regs(struct mpu_data *ret_data)
{
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