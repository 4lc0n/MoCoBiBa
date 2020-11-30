/*  Motion Controlled Bicicle Backlight
// Created: 24.11.2020
// 22:00: Interrupt works
// 
// TODO: Power reduction
// TODO: Sleep-Mode
// TODO: read Accel fro ACCEL_Z
// TODO: Monitor Battery Voltage
// TODO: Monitor Light Conditions
// TODO: Disable Brownout Detection, if active
// TODO: low battery warning
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "usiTwiMaster.h"



#define F_CPU 1000000UL

// register in order to set to cycle and interrupt on accel_x
const uint8_t lp_regs[8]  = {107, 108, 29, 56, 57, 31, 30, 107};
const uint8_t lp_data[8] = {1, 31, 1, 64, 192, 16, 4, 0x29};

const uint8_t sp_regs[9] = {107, 108, 28, 29, 56, 57, 31, 30, 107};
const uint8_t sp_data[9] = {1, 31, 1, 0,  64, 192, 16, 8, 0x9};



// variables determing current states:
bool darkness = false;
bool motion = false;
volatile bool sleep = false;

// function declarations
void setup_watchdog();
void setup_pins();
void enter_sleep_wdt();

bool mpu_set_register(uint8_t reg, uint8_t data);
bool mpu_get_register(uint8_t reg, uint8_t* data);
bool mpu_set_registers(const uint8_t *reg, const uint8_t *data, uint8_t number);
bool mpu_get_registers(uint8_t reg, uint8_t *data, uint8_t number);
bool mpu_set_interrupt();
bool mpu_set_sampling();
bool mpu_read_regs(struct mpu_data *ret_data);
uint16_t read_v_batt();
uint16_t read_v_light();



int main()
{
  uint8_t temp = 0;
  setup_pins();
  setup_watchdog();
  for(temp = 0; temp < 3; temp++){
    LED_PORT |= (0x01 << LED);
    _delay_ms(100);
    LED_PORT &= ~(0x01 << LED);
    _delay_ms(100);
  }
 
  // usiTwiMasterInitialize();
  while(1){
    check_for_darkness();
    if(darkness)
    {
      LED_PORT |= (0x01 << LED);

    }
    else
    {
      LED_PORT &= ~(0x01 << LED);
    }

    enter_sleep_wdt();
  }
  


}

void setup_watchdog()
{
  // set prescaler to timeout of 1 sec
  // set to generate interrupt instead of reset
  cli();
  wdt_reset();
  MCUSR &= ~(0x01 << WDRF);
  WDTCSR |= (0x01 << WDCE) | (0x01 << WDE);
  WDTCSR |=   (0x00 << WDP3) | (0x01 << WDP2) | (0x01 << WDP1) | (0x00 << WDP0);
  WDTCSR |= (0x01 << WDIE);
  sei();
}

void check_for_darkness()
{
  uint16_t light_val;

  // save satate of LED_PORT
  uint8_t temp = LED_PORT;
  // disable LEDS
  LED_PORT &= ~(0x01 << LED);

  //get sample
  light_val = read_v_light();
  if(!darkness && light_val > DARKNESS_THRESHHOLD_UPPER)
  {
    // v_light above threshhold: it is getting dark (R2 is photoresistor)
    darkness = true;
  }
  else if(darkness && light_val < DARKNESS_THRESHHOLD_LOWER)
  {
    // v_light below lower threshhold: its bright again
    darkness = false;
  }
}

void enter_sleep_wdt()
{
  sleep = true;

  // go to sleep
  MCUCR &= ~((0x01 << SM1) | (0x01 << SM0) | (0x01 << SE));
  MCUCR |= (0x01 << SM1) | (0x00 << SM0);
  
  // needs optimisation for power consumption: disable ports, peripheries, mpu, ...

  // disable brown out detection and go to sleep
  // uint8_t temp = MCUCR;
  // MCUCR |= (0x01 << BODS) | (0x01 << BODSE);
  // MCUCR = temp | (0x01 << BODS);
  MCUCR |= (0x01 << SE);
  while(sleep == true)
    asm volatile ("nop");
}

void setup_pins()
{


  // set all unconnected pins to a defined level: input, pullup
  DDRA = 0;
  DDRB = 0;

  PORTA = 0xFF;
  PORTB = 0x0F;


  // debug pins: PB0 and PB1

  DEBUG0_DDR |= (0x01 << DEBUG0);
  DEBUG1_DDR |= (0x01 << DEBUG1);

  // set debug pins to low
  DEBUG0_PORT &= ~(0x01 << DEBUG0);
  DEBUG1_PORT &= ~(0x01 << DEBUG1);

  // set LED as output, set low
  LED_DDR |= (0x01 << LED);
  LED_PORT &= ~(0x01 << LED);

  // set acc_int pin as input, no pullup
  ACC_INT_DDR &= ~(0x01 << ACC_INT);
  ACC_INT_PORT &= ~(0x01 << ACC_INT);

  // set voltage pins as no pullup, digital interface disabled
  V_BATT_PORT &= ~(0x01 << V_BATT);
  V_LIGHT_PORT &= ~(0x01 << V_LIGHT);
  DIDR0 |= (0x0 << V_BATT) | (0x01 << V_LIGHT);

  // set V_SINK as input to prevent excessive current flow, disable pullup
  V_SINK_DDR &= ~(0x01 << V_SINK);
  V_SINK_PORT &= ~(0x01 << V_SINK);

  

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
    _delay_us(10);
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
  _delay_ms(100);

  // for(uint8_t i = 0; i < 9; i++)
  // {
  //   if(!mpu_set_register(sp_regs[i], sp_data[i]))
  //   {
  //     //failed to transmit
  //     return false;
  //   }
  //   else{
  //     //wait for a short period of time to distinquish the data packets on the logic analyzer
  //     _delay_us(100);
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


uint16_t read_v_batt()
{

  const uint8_t num_of_conversions = 10;
  uint8_t counter = 0;
  uint16_t data;

  // set sink pin to gnd
  V_SINK_DDR |= (0x01 << V_SINK);
  V_SINK_PORT &= ~(0x01 << V_SINK);

  // enable and configure adc
  
  // enable ADC by setting ADEN in ADCSRA
  ADCSRA |= (0x01 << ADEN);
  
  // division factor: 16
  ADCSRA &= ~(0x07);    // clear prescaler setting
  ADCSRA |= (0x04);

  // select voltage reference: VCC (ADMUX [7:6] = 0)
  // select adc channel: ADC1 (ADMUX[5:0] = 1)
  ADMUX = 0x00 | 0x01;
  
  // wait for 1 ms to let the resistor cicruit settle in
  _delay_ms(1);

  // start first conversion
  ADCSRA |= (0x01 << ADSC);

  // wait for conversion to complete
  while(ADCSRA & (0x01 << ADSC)){
    asm volatile ("nop");
  }
  data = ADC;
  
  // read adc for at least 10 times
  for(counter = 0; counter < num_of_conversions; counter++)
  {
    // start conversion
    ADCSRA |= (0x01 << ADSC);

    // wait for conversion to complete
    while(ADCSRA & (0x01 << ADSC)){
      asm volatile ("nop");
    }
    data += ADC;
  }

  // take average of 10 measurements
  data = data / num_of_conversions;



  // disable adc by clearing ADEN in ADCSRA
  ADCSRA &= ~(0x01 << ADEN);

  // set sink as input
  V_SINK_DDR &= ~(0x01 << V_SINK);



  // return raw value
  return data;

}


uint16_t read_v_light()
{
  const uint8_t num_of_conversions = 10;
  uint8_t counter = 0;
  uint16_t data;

  // set sink pin to gnd
  V_SINK_DDR |= (0x01 << V_SINK);
  V_SINK_PORT &= ~(0x01 << V_SINK);

  // enable and configure adc
  
  // enable ADC by setting ADEN in ADCSRA
  ADCSRA |= (0x01 << ADEN);
  
  // division factor: 16
  ADCSRA &= ~(0x07);    // clear prescaler setting
  ADCSRA |= (0x04);

  // select voltage reference: VCC (ADMUX [7:6] = 0)
  // select adc channel: ADC1 (ADMUX[5:0] = 3)
  ADMUX = 0x00 | 0x03;
  
  // wait for 1 ms to let the resistor cicruit settle in
  //_delay_ms(1);

  // run 3 times num_of conversion to get sample-and-hold segment to current level: high impedance input signals
  for(counter = 0; counter < num_of_conversions * 3; counter++){
    // start first conversion
    ADCSRA |= (0x01 << ADSC);

    // wait for conversion to complete
    while(ADCSRA & (0x01 << ADSC)){
      asm volatile ("nop");
    }
    data = ADC;
    _delay_us(3);
  }
  
  // read adc for at least 10 times
  for(counter = 0; counter < num_of_conversions; counter++)
  {
    // start conversion
    ADCSRA |= (0x01 << ADSC);

    // wait for conversion to complete
    while(ADCSRA & (0x01 << ADSC)){
      asm volatile ("nop");
    }
    data += ADC;
  }

  // take average of 10 measurements
  data = data / num_of_conversions;



  // disable adc by clearing ADEN in ADCSRA
  ADCSRA &= ~(0x01 << ADEN);

  // set sink as input
  V_SINK_DDR &= ~(0x01 << V_SINK);



  // return raw value
  return data;
}


ISR(WDT_vect)
{
  // reconfigure registers for interrupt
  WDTCSR |= (0x01 << WDIE) ;

  sleep = false;
}