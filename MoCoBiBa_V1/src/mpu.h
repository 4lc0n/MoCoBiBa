#ifndef MPU_H
#define MPU_H


#include <stdint.h>
#include <stdbool.h>

#include "usiTwiMaster.h"

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
#define MPU_WAI   0x70

bool mpu_set_register(uint8_t reg, uint8_t data);
bool mpu_get_register(uint8_t reg, uint8_t* data);
bool mpu_set_registers(const uint8_t *reg, const uint8_t *data, uint8_t number);
bool mpu_get_registers(uint8_t reg, uint8_t *data, uint8_t number);
bool mpu_set_interrupt();
bool mpu_set_sampling();
bool mpu_set_sleep();
bool mpu_read_regs(struct mpu_data *ret_data);
bool mpu_get_wai();



#endif