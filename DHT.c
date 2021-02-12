#include "DHT.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdbool.h>
#include "io_helper.h"
#include "time.h"

#define PORT  D
#define PIN   2

typedef enum
{
  ST_IDLE,
  ST_ERROR,
  ST_ERROR_CHECKSUM,
  ST_BIT_CLEARED,
  ST_WAIT_FOR_RESPONSE,
  ST_RESPONSE_STARTED,
  ST_RESPONSE_COMPLETED,
  ST_BYTE_TRANSFER_STARTING,
  ST_BIT_TRANSFER_STARTING,
  ST_BIT_INCOMING,
  ST_COMPLETED
} State;

uint8_t dht_humidity = 255;
uint8_t dht_temperature = 255;

static volatile State state_;
static volatile uint8_t timer_counter_;
static volatile uint8_t byte_index_;
static volatile uint8_t bit_index_;

static volatile uint8_t data_[5];

bool process_data();
uint8_t read_sensor();

void on_second();

void dht_init()
{
  state_ = ST_IDLE;
  //TCCR0 = (1 << WGM01); // Timer CTC mode.
  time_subscribe_s(&on_second);
}

bool process_data()
{
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < 4; ++i)
  checksum += data_[i];

  if (data_[4] != checksum)
  {
    state_ = ST_ERROR_CHECKSUM;
    return false;
  }

  dht_humidity = data_[0] + data_[1] * 0.1;
  dht_temperature = data_[2] + data_[3] * 0.1;
  return true;
}

uint8_t read_sensor()
{
  //TCCR0 = 0x02;
  //TCNT0 = 0;
  //dht_humidity = TCNT0;
  //if (PINx(PORT, PIN))
    //dht_temperature = dht_humidity;
  ////else
    ////dht_temperature = TCNT0;
  //dht_temperature = TCNT0;
  //return 2;

  dht_temperature = 0;
  uint8_t cnt = 0;/*, check*/;
  int8_t i,j;
    
  /******************* Sensor communication start *******************/
    
  /* Set data pin as output first */
  IO_DIRECTION_OUT(PORT, PIN);
    
  /* First we need milliseconds delay, so set clk/1024 prescaler */
  TCCR0 = 0x05;
  TCNT0 = 0;

  /* Clear bit for 20 ms */
  PORTx_OFF(PORT, PIN);
    
  /* Wait about 20 ms */
  while(TCNT0 < 144);
    
  /* Now set Timer0 with clk/8 prescaling.
    Gives 1µs per cycle @8Mhz */
  TCCR0 = 0x02;
  TCNT0 = 0;
    
  /* Pull high again */
  PORTx_ON(PORT, PIN);
    
  /* And set data pin as input */
  IO_DIRECTION_IN(PORT, PIN);
  /* Wait for response from sensor, 20-40µs according to datasheet */
  while(PINx(PORT, PIN))
  { cnt = TCNT0; if (cnt >= 57) { dht_temperature = cnt; return 1; } }
    
  /************************* Sensor preamble *************************/
    
  TCNT0 = 0;
    
  /* Now wait for the first response to finish, low ~80µs */
  while(!PINx(PORT, PIN))
  { cnt = TCNT0; if (cnt >= 93) { dht_temperature = cnt; return 2; } }
    
  TCNT0 = 0;
    
  /* Then wait for the second response to finish, high ~80µs */
  while(PINx(PORT, PIN))
  { cnt = TCNT0; if (cnt >= 93) { dht_temperature = cnt; return 3; } }

  TCNT0 = 0;

  /********************* Data transmission start **********************/
    
  for (i = 0; i < 5; ++i)
  {
    for(j = 7; j >= 0; --j)
    {
      //TCNT0 = 0;
      
      /* First there is always a 50µs low period */
      while(!PINx(PORT, PIN))
      { cnt = TCNT0; if (cnt >= 56) { dht_temperature = cnt; return 4; } }
            
      TCNT0 = 0;
            
      /* Then the data signal is sent. 26 to 28µs (ideally)
        indicate a low bit, and around 70µs a high bit */
      while(PINx(PORT, PIN))
      { cnt = TCNT0; if (cnt >= 78) { dht_temperature = cnt; return 5; } }

      TCNT0 = 0;
            
      /* Store the value now so that the whole checking doesn't
        move the TCNT0 forward by too much to make the data look
        bad */
      //cnt = TCNT0;
        
//      if (cnt >= 19 && cnt <= 31)
      if (cnt <= 41)
        data_[i] &= ~(1 << j);
      //else if (cnt >= 51 && cnt <= 78)
      else if (cnt<= 78)
        data_[i] |= (1 << j);
      else
      {
        dht_temperature = cnt;
        return 6;
      }
    }
  }

  return 0;
    
  /********************* Sensor communication end *********************/
  
  //check = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
  //
  //if (check != data[4]) return 0;
  //
  //for(i = 0; i < 4; ++i)
  //{ arr[i] = data[i]; }
  //
  //return 1;
}
#include <util/delay.h>
void on_second()
{

  if (time_s % 5 == 0)
  {
  //_delay_ms(350);
  //read_sensor();
    dht_humidity = read_sensor();

    //if (dht_humidity == 0)
      //process_data();
  }
}

//ISR(TIMER0_COMP_vect)
//{
  //timer_counter_++;
//
  //switch (state_)
  //{
    //case ST_BIT_CLEARED:
    //// Set pin as input.
    //PORTx_ON(PORT, PIN);
    //IO_DIRECTION_IN(PORT, PIN);
//
    //// Wait for response to start.
    //state_ = ST_WAIT_FOR_RESPONSE;
    //timer_counter_ = 0;
    //TCCR0 = (1 << WGM01) | (1 << CS01); // Set F_CPU/8 prescaler.
    ////TCNT0 = 0;
    //OCR0 = 2;
    ////TIMSK &= ~(1 << OCIE0);
    //break;
//
    //case ST_WAIT_FOR_RESPONSE:
    //// Wait for pin to become unset (40 µs max).
    //if (timer_counter_ >= 37) { /*dht_humidity++; */state_ = ST_ERROR; break; } // Timeout at 40 µs.
//
    //if (!PINx(PORT, PIN))
    //{
      //dht_humidity = timer_counter_;
      //// Response started. Wait for response to finish.
      //state_ = ST_RESPONSE_STARTED;
      //timer_counter_ = 0;
      //TCNT0 = 0;
    //}
//
    //break;
     //
    //case ST_RESPONSE_STARTED:
    //// Wait for pin to become set (80 µs ideally).
    //if (timer_counter_ >= 93) { dht_humidity = 102; state_ = ST_ERROR; break; } // Timeout at 100 µs.
//
    //if (PINx(PORT, PIN))
    //{
      //dht_temperature = timer_counter_;
      //// Response completed. Wait for data transfer start.
      //state_ = ST_RESPONSE_COMPLETED;
      //timer_counter_ = 0;
      //TCNT0 = 0;
      //TIMSK &= ~(1 << OCIE0);
    //}
//
    //break;
//
    //case ST_RESPONSE_COMPLETED:
    //// Wait for pin to become unset again (80 µs ideally).
    //if (timer_counter_ >= 93) { dht_humidity = 103; state_ = ST_ERROR; break; } // Timeout at 100 µs.
//
    //if (!PINx(PORT, PIN))
    //{
      //// Data transfer is about to start.
      //state_ = ST_BYTE_TRANSFER_STARTING;
      //timer_counter_ = 0;
      //byte_index_ = 0;
      //TCNT0 = 0;
    //}
//
    //break;
//
    ///** Single byte processing. */
//
    //case ST_BYTE_TRANSFER_STARTING:
    //if (byte_index_ > 5)
    //{
      //if (process_data())
      //state_ = ST_COMPLETED;
//
      //break;
    //}
//
    //bit_index_ = 0;
    //state_ = ST_BIT_TRANSFER_STARTING;
//
    //case ST_BIT_TRANSFER_STARTING:
    //// Wait during low period for bit start (50 µs ideally).
    //if (timer_counter_ >= 56) { dht_humidity = 104; state_ = ST_ERROR; break; } // Timeout at 60 µs.
//
    //if (!PINx(PORT, PIN))
    //break;
     //
    //// Data incoming.
    //state_ = ST_BIT_INCOMING;
    //timer_counter_ = 0;
    //TCNT0 = 0;
    //break;
//
    //case ST_BIT_INCOMING:
    //// Wait for bit end. 26 to 28 µs indicate low bit, 70 µs indicate high bit.
    //if (timer_counter_ >= 78) { dht_humidity = 105; state_ = ST_ERROR; break; } // Timeout at 84 µs.
//
    //if (PINx(PORT, PIN))
    //break;
//
    //// Bit ended.
    //if (timer_counter_ >= 19 && timer_counter_ <= 31) // Between 20.80 and 33.60 µs.
    //data_[byte_index_] &= ~(1 << bit_index_);
    //else if (timer_counter_ >= 51 && timer_counter_ <= 78) // Between 56 and 84 µs.
    //data_[byte_index_] |= (1 << bit_index_);
    //else
    //{
      ////dht_humidity = 106;
      ////dht_temperature = timer_counter_;
      //state_ = ST_ERROR; // Erroneous bit.
      //break;
    //}
//
    //if (++bit_index_ >= 8)
    //{
      //byte_index_++;
      //state_ = ST_BYTE_TRANSFER_STARTING;
    //}
//
    //timer_counter_ = 0;
    //TCNT0 = 0;
    //break;
//
    //default:
    //break;
  //}
//}
