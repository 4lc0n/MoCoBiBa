#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>





#define DEBUG0 PB0
#define DEBUG0_PORT PORTB
#define DEBUG0_DDR DDRB
#define DEBUG1 PB1
#define DEBUG1_PORT PORTB
#define DEBUG1_DDR DDRB
#define LED PB2
#define LED_PORT PORTB
#define LED_DDR DDRB
#define LED_PIN PINB

#define ACC_INT PA0
#define ACC_INT_PORT PORTA
#define ACC_INT_DDR DDRA
#define ACC_INT_PIN PINA

#define V_BATT PA1
#define V_BATT_PIN PINA
#define V_BATT_PORT PORTA
#define V_BATT_DDR DDRA
#define V_SINK PA2
#define V_SINK_PIN PINA
#define V_SINK_PORT PORTA
#define V_SINK_DDR DDRA
#define V_LIGHT PA3
#define V_LIGHT_PIN PINA
#define V_LIGHT_PORT PORTA
#define V_LIGHT_DDR DDRA



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


// definitions for configuration

#define DARKNESS_THRESHHOLD_UPPER 512
#define DARKNESS_THRESHHOLD_LOWER 440

#define MOTION_THRESHHOLD         0.2

#define MOTION_TIMEOUT            60



#endif