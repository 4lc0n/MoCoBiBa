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
#include "mpu.h"









// variables determing current states:
bool darkness = false;
volatile uint8_t darkness_lp = 0;
volatile bool motion = false;
volatile uint8_t motion_lp = 0;
volatile bool sleep = false;
volatile uint8_t last_motion = 0;
enum error_code {NO_ERROR, MPU_ERROR, BATTERY_ERROR};
enum error_code error = NO_ERROR;


uint16_t u16_v_light;
enum modes {sleepMode, interruptMode, samplingMode};
enum modes mpu_mode = sleepMode;
// function declarations
void setup_watchdog(uint8_t wdtimeout);
void setup_pins();
void setup_pwm();
void disable_pwm();
void enter_sleep_wdt(uint8_t wdtimeout);
void setup_interrupt_pin();
void disable_interrupt_pin();

bool check_for_darkness(bool ndarkness, uint16_t *nv_light);


uint16_t read_v_batt();
uint16_t read_v_light();



int main()
{
  uint8_t temp = 0;
  setup_pins();
  setup_watchdog(WDTO_8S);
  
  _delay_ms(1000);
 
  usiTwiMasterInitialize();
  
  if(!mpu_set_sleep() || !mpu_get_wai())
  {
    error = MPU_ERROR;
    LED_PORT |= (0x01 << LED);
    _delay_ms(500);
    LED_PORT &= ~(0x01 << LED);
    _delay_ms(100);
  }
  
 for(temp = 0; temp < 3; temp++){
    LED_PORT |= (0x01 << LED);
    _delay_ms(100);
    LED_PORT &= ~(0x01 << LED);
    _delay_ms(100);
  }
  while(1){
    if(check_for_darkness(darkness, &u16_v_light))
    {
      if(darkness_lp < 5)
      {
        darkness_lp++;
      }

      if(darkness_lp > 3) 
      { // if last 3 samples were positive
        darkness = true;
      }
      
    }
    else
    {
      if(darkness_lp > 0)
      {
        darkness_lp --;
      }
      if(darkness_lp == 0)
      {
        darkness = false;
      }
    }
    

    
    if(darkness)
            DEBUG1_PORT |= (0x01 << DEBUG1);

    // if motion and darkness
    if( motion && darkness)
    {
      

      if(mpu_mode != samplingMode && !(error == MPU_ERROR))
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
        motion_lp = 0;
      }      
      // TODO: implement sampling and filetring as q15
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
        if(mpu_mode != interruptMode && !(error == MPU_ERROR))
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
          enter_sleep_wdt(WDTO_1S);
        }
      }

      // if not motion and not darkness
      else
      {
        disable_interrupt_pin();
        if(mpu_mode != sleepMode&& !(error == MPU_ERROR))
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
        enter_sleep_wdt(WDTO_8S);

      }
    }
  }
}

void setup_watchdog(uint8_t wdtimeout)
{
  // set prescaler to timeout of wdtimeout seconds
  // set to generate interrupt instead of reset
  cli();
  wdt_reset();
  MCUSR &= ~(0x01 << WDRF);
  WDTCSR = 0;
  WDTCSR |= (0x01 << WDCE) | (0x01 << WDE);
  WDTCSR |=   (0x00 << WDP3) | (0x01 << WDP2) | (0x01 << WDP1) | (0x00 << WDP0);
  WDTCSR |= wdtimeout;
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

void enter_sleep_wdt(uint8_t wdtimeout)
{
  setup_watchdog(wdtimeout);
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
  cli();

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


  // clear interrupt flag for PCINT0
  GIFR |= (0x01 << PCIF0);
  sei();

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

  // select voltage reference: 1.1 voltage reference (ADMUX [7:6] = 2)
  // select adc channel: ADC1 (ADMUX[5:0] = 1)
  ADMUX = 0;
  ADMUX = (0x01 << REFS1) | 0x01;
  
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
    if(motion_lp < 5)
    {
      motion_lp ++;
    }
    // if 3 times positive
    if(motion_lp > 2)
    {
      motion = true;
      last_motion = MOTION_TIMEOUT;
    }
    
    
  }
}