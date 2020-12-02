/*  Motion Controlled Bicicle Backlight
// Created: 24.11.2020
// 22:00: Interrupt works

// 02.12.
// State machine works, sort of
// BUG: by some chance pin change interrupt on accel_pin always triggers
// 
// 
// DONE: Power reduction
// DONE: Sleep-Mode
// TODO: read Accel fro ACCEL_Z
// TODO: Monitor Battery Voltage
// DONE: Monitor Light Conditions
// DONE: Disable Brownout Detection, if active
// TODO: low battery warning
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "usiTwiMaster.h"





// register in order to set to cycle and interrupt on accel_z // 105 was 57 (dunno why)
const uint8_t lowpower_regs[8]  = {107, 108, 29,   56, 105,  31, 30, 107};      // only axis x activated, wake on motion activated, sampling at  4 Hz
const uint8_t lowpower_data[8] = {   1,0x37,  5, 0x40,0xC0,0x0B,  4, 0x29};     // threshhold at reg 31

// registers in order to set to sampling at 125 Hz, all 3 accel axes
const uint8_t sampling_regs[9] = {107, 108, 28, 29,  56, 105,   31,  30, 107};   // wake on motion activated, sample-rate: 125, +-2g, gyro deactivated
const uint8_t sampling_data[9] = {  1,0x07,  1,  0,0x40,0xC0, 0x0B,0x08, 0x09};  // disable 

const uint8_t sleep_regs[3] = {108, 107, 107};
const uint8_t sleep_data[3] = {0x3F, 0x08, 0x40};

// variables determing current states:
bool darkness = false;
volatile bool motion = false;
volatile bool sleep = false;
volatile uint8_t last_motion = 0;


uint16_t u16_v_light;
enum modes {sleepMode, interruptMode, samplingMode};
enum modes mpu_mode = sleepMode;
// function declarations
void setup_watchdog();
void setup_pins();
void setup_pwm();
void disable_pwm();
void enter_sleep_wdt();
void setup_interrupt_pin();
void disable_interrupt_pin();

bool check_for_darkness(bool ndarkness, uint16_t *nv_light);

bool mpu_set_register(uint8_t reg, uint8_t data);
bool mpu_get_register(uint8_t reg, uint8_t* data);
bool mpu_set_registers(const uint8_t *reg, const uint8_t *data, uint8_t number);
bool mpu_get_registers(uint8_t reg, uint8_t *data, uint8_t number);
bool mpu_set_interrupt();
bool mpu_set_sampling();
bool mpu_set_sleep();
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
  _delay_ms(1000);
 
  usiTwiMasterInitialize();
  mpu_set_sleep();
  
 
  while(1){
    darkness = check_for_darkness(darkness, &u16_v_light);
    
    if(darkness)
            DEBUG1_PORT |= (0x01 << DEBUG1);

    // if motion and darkness
    if( motion && darkness)
    {
      

      if(mpu_mode != samplingMode)
      {
        usiTwiMasterInitialize();
        if(!mpu_set_sampling())
        {
          mpu_set_sampling();
          DEBUG0_PORT |= (0x01 << DEBUG0);
        }
        mpu_mode = samplingMode;
        setup_pwm();
      }



      
      if(last_motion == 0)
      {
        // timed out: no motion in last 60 sec
        motion = false;
        disable_pwm();
      }      

      // sample the mpu

      // interprete data with a lpfilter

      // modify timer to either signal braking light or normal light
      // led pin: OC0A

      // if breaking: 
      // OCR0A = 255;

      // else:
        OCR0A = 60;

      // delay for approx. 125 Hz loop time
      _delay_ms(100);

    }

    // if not motion, but darkness
    else {
      if (darkness)
      {
        setup_interrupt_pin();
        // if mpu not configured yet
        if(mpu_mode != interruptMode)
        {
          usiTwiMasterInitialize();
          if(!mpu_set_interrupt())
          { // try again!
            mpu_set_interrupt();
            DEBUG0_PORT |= (0x01 << DEBUG0);
          }
          mpu_mode = interruptMode;
        }
        

        // sleep for 1 ms
        else
        {
          enter_sleep_wdt();
        }
      }

      // if not motion and not darkness
      else
      {
        disable_interrupt_pin();
        if(mpu_mode != sleepMode)
        {
          usiTwiMasterInitialize();
          if(!mpu_set_sleep())
          { // try again!
            mpu_set_sleep();
            DEBUG0_PORT |= (0x01 << DEBUG0);
          }
          mpu_mode = sleepMode;
        }

        // sleep for 1 ms
        enter_sleep_wdt();

      }
    }
    


    // if(darkness)
    // {
    //   // if motion is set to true: sample should already be configured
    //   if(motion)
    //   {
    //     // 
    //     if(ACC_INT_PIN & (0x01 << ACC_INT))
    //     {
    //       last_motion = 60;   // reset last_motion to 60 -> 1 minute timer, reduced by WDT
          
    //     }
              

    //     // sample the mpu

    //     // interprete data with a lpfilter

    //     // modify timer to either signal braking light or normal light
    //     // led pin: OC0A

    //     // if breaking: 
    //     // OCR0A = 255;

    //     // else:
    //      OCR0A = 60;


    //     if(!last_motion)
    //     {
    //       motion = false;
    //       mpu_mode = interruptMode;
    //       mpu_set_interrupt();
    //       disable_pwm();
    //       LED_DDR &= ~(0x01 << LED);
    //     }
    //   }


    //   // else: motion is false, it is only dark: set interrupt
    //   // if motion: go to upper part
    //   else {    // motion
    //     // if interrupt not set
    //     if(mpu_mode != interruptMode)
    //     {
    //       mpu_set_interrupt();
    //       mpu_mode = interruptMode;
    //     }
        
    //     // if interrupt pin reads as one
    //     if(ACC_INT_PIN & (0x01 << ACC_INT))
    //     {


    //       // set motion to true
    //       motion = true;
    //       // activate mpu to sample at 80 Hz
    //       mpu_mode = samplingMode;
    //       mpu_set_sampling();
    //       last_motion = 60;

    //       // enable led 
    //       // todo: implement timer for pwm
    //       setup_pwm();
    //       LED_DDR |= (0x01 << LED);
    //     }
        
        
    //   }
    // }
    // else
    // {
    //   if(mpu_mode != sleepMode)
    //   {
    //     mpu_set_sleep();
    //     mpu_mode = sleepMode;
    //   }
      
    //   motion = false;
      
    // }
    
    // if(!motion)
    // {
    //    enter_sleep_wdt();
    // }
    // else
    // {
    //   _delay_ms(1000);
    //   if(last_motion > 0) last_motion--; // reduce last_motion with every interrupt; after 60 seconds of no motion: turn lights off

    // }
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

bool check_for_darkness(bool ndarkness, uint16_t *nv_light)
{
  uint16_t light_val;

  // if braking is active: measuring light conditions is useless: assume it is dark
  if(OCR0A == 255)
  {
    return ndarkness;
  }

  // if motion: wait for pulse to pass, then sample
  if(motion & (TCNT0 < OCR0A))
  {
    while((TCNT0 < OCR0A));  
  }
  // save satate of LED_PORT
  uint8_t temp = LED_DDR;
   // disable LEDS
  LED_DDR &= ~(0x01 << LED);
  //get sample
  light_val = read_v_light();

  LED_DDR = temp;

  *nv_light = light_val;

  

  if(!ndarkness && light_val > DARKNESS_THRESHHOLD_UPPER)
  {
    // v_light above threshhold: it is getting dark (R2 is photoresistor)
    return true;
  }
  else if(ndarkness && light_val < DARKNESS_THRESHHOLD_LOWER)
  {
    // v_light below lower threshhold: its bright again
    return false;
    
  }
  else
  {
    // if no threshhold was passed: return old state
    return ndarkness;
  }
  
  
}

void enter_sleep_wdt()
{
  sleep = true;

  // disable pins for adc
  DDRA &= ~(0xFE);
  DDRB &= ~(0x0C);



  // go to sleep
  MCUCR &= ~((0x01 << SM1) | (0x01 << SM0) | (0x01 << SE));
  MCUCR |= (0x01 << SM1) | (0x00 << SM0);
  
  // needs optimisation for power consumption: disable ports, peripheries, mpu, ...
  // disable  timer 1, usi and adc
  // keep timer0 alive
  uint8_t prr_old = PRR;
  PRR |= (0x01 << PRTIM0) | (0x01 << PRTIM1) | (0x01 << PRUSI) | (0x01 << PRADC);



  // disable brown out detection and go to sleep 
  uint8_t temp = MCUCR;
  MCUCR |= (0x01 << BODS) | (0x01 << BODSE);
  MCUCR = temp | (0x01 << BODS);
  MCUCR |= (0x01 << SE);
  
  sleep_cpu();

  while(sleep == true)
    asm volatile ("nop");

  PRR = prr_old;
  setup_pins();
  
}

void setup_pins()
{


  // set all unconnected pins to a defined level: input, pullup
  DDRA = 0;
  DDRB = 0;

  PORTA = 0xAE;   // set to pullup: A1, A2, A3, A5, A7
  PORTB = 0x08;


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


void setup_pwm()
{

  LED_DDR |= (0x01 << LED);
  // f_clk = 1e6 Hz
  // prescaler: 64
  // f_timer: 61 Hz
  // enable timer
  PRR &= ~(0x01 << PRTIM0);

  // enable Output Compare 0 A
  // fast pwm mode
  TCCR0A |= (0x01 << COM0A1) | (0x01 << WGM01) | (0x01 << WGM00);
  TCCR0B |= (0x01 << CS01) | (0x01 << CS00);

  // set compare unit to mid-range
  OCR0A = 128;

  // no interrupt needed

}


void disable_pwm()
{
  // disable port function
  TCCR0A &= ~((0x01 << COM0A1) | (0x01 << COM0A0));
  TCCR0B = 0;
  // disable timer in power reduction register
  PRR |= (0x01 << PRTIM0);

  LED_DDR &= ~(0x01 << LED);

}


void setup_interrupt_pin()
{
  // clear Interrupt flag
  GIFR |= (0x01 << PCIF0);
  // enable interrupt on PCInt 1 Vector
  GIMSK |= (0x01 << PCIE0);


  PCMSK0 |= (0x01 << PCINT0);

}


void disable_interrupt_pin()
{
  GIMSK &= ~(0x01<< PCIE0);
  PCMSK0 &= ~(0x01 << PCINT0);
}


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
  const uint8_t num_of_conversions = 5;
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
  for(counter = 0; counter < num_of_conversions/3; counter++){
    // start first conversion
    ADCSRA |= (0x01 << ADSC);

    // wait for conversion to complete
    while(ADCSRA & (0x01 << ADSC)){
      asm volatile ("nop");
    }
    data = ADC;
    // _delay_us(2);
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
  MCUCR &= ~(0x01 << SE);

  if(last_motion > 0) last_motion--; // reduce last_motion with every interrupt; after 60 seconds of no motion: turn lights off
  sleep = false;
}

ISR(PCINT0_vect)
{
  if(ACC_INT_PIN & (0x01 << ACC_INT))
  {
    // pin rises
    motion = true;
    last_motion = MOTION_TIMEOUT;
  }
}