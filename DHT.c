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

volatile float dht_humidity = 25.6;
volatile float dht_temperature = 36.7;

static volatile State state_;
static volatile uint8_t timer_counter_;
static volatile uint8_t byte_index_;
static volatile uint8_t bit_index_;

static volatile uint8_t data_[5];

bool process_data();
void read_data();

void on_second();

void dht_init()
{
  state_ = ST_IDLE;
  TCCR0 = (1 << WGM01); // Timer CTC mode.
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

void read_data()
{
  // Set pin as output and low.
  IO_DIRECTION_OUT(PORT, PIN);
  PORTx_OFF(PORT, PIN);

  // Set up timer.
  state_ = ST_BIT_CLEARED;
  TCCR0 = (1 << WGM01) | (1 << CS02) | (1 << CS00); // Set F_CPU/1024 prescaler.
  OCR0 = 144; // Wait for 20 ms.
  TCNT0 = 0;
  TIFR |= (1 << OCF0);
  TIMSK |= (1 << OCIE0); // Enable CTC interrupt for the timer.
  TIFR |= (1 << OCF0);
  sei(); // Enable global interrupts.
}

void on_second()
{
  //if (time_s % 5 == 0)
    read_data();
}

ISR(TIMER0_COMP_vect)
{
  timer_counter_++;

  switch (state_)
  {
    case ST_BIT_CLEARED:
      // Set pin as input.
      PORTx_ON(PORT, PIN);
      IO_DIRECTION_IN(PORT, PIN);

      // Wait for response to start.
      state_ = ST_WAIT_FOR_RESPONSE;
      timer_counter_ = 0;
      TCCR0 = (1 << WGM01) | (1 << CS01); // Set F_CPU/8 prescaler.
      //TCNT0 = 0;
      OCR0 = 2;
      //TIMSK &= ~(1 << OCIE0);
      break;

    case ST_WAIT_FOR_RESPONSE:
      // Wait for pin to become unset (40 µs max).
      if (timer_counter_ >= 37) { /*dht_humidity++; */state_ = ST_ERROR; break; } // Timeout at 40 µs.

      if (!PINx(PORT, PIN))
      {
        dht_humidity = timer_counter_;
        // Response started. Wait for response to finish.
        state_ = ST_RESPONSE_STARTED;
        timer_counter_ = 0;
        TCNT0 = 0;
      }

      break;
    
    case ST_RESPONSE_STARTED:
      // Wait for pin to become set (80 µs ideally).
      if (timer_counter_ >= 93) { dht_humidity = 102; state_ = ST_ERROR; break; } // Timeout at 100 µs.

      if (PINx(PORT, PIN))
      {
        dht_temperature = timer_counter_;
        // Response completed. Wait for data transfer start.
        state_ = ST_RESPONSE_COMPLETED;
        timer_counter_ = 0;
        TCNT0 = 0;
        TIMSK &= ~(1 << OCIE0);
      }

      break;

    case ST_RESPONSE_COMPLETED:
      // Wait for pin to become unset again (80 µs ideally).
      if (timer_counter_ >= 93) { dht_humidity = 103; state_ = ST_ERROR; break; } // Timeout at 100 µs.

      if (!PINx(PORT, PIN))
      {
        // Data transfer is about to start.
        state_ = ST_BYTE_TRANSFER_STARTING;
        timer_counter_ = 0;
        byte_index_ = 0;
        TCNT0 = 0;
      }

      break;

    /** Single byte processing. */

    case ST_BYTE_TRANSFER_STARTING:
      if (byte_index_ > 5)
      {
        if (process_data())
          state_ = ST_COMPLETED;

        break;
      }

      bit_index_ = 0;
      state_ = ST_BIT_TRANSFER_STARTING;

    case ST_BIT_TRANSFER_STARTING:
      // Wait during low period for bit start (50 µs ideally).
      if (timer_counter_ >= 56) { dht_humidity = 104; state_ = ST_ERROR; break; } // Timeout at 60 µs.

      if (!PINx(PORT, PIN))
        break;
      
      // Data incoming.
      state_ = ST_BIT_INCOMING;
      timer_counter_ = 0;
      TCNT0 = 0;
      break;

    case ST_BIT_INCOMING:
      // Wait for bit end. 26 to 28 µs indicate low bit, 70 µs indicate high bit.
      if (timer_counter_ >= 78) { dht_humidity = 105; state_ = ST_ERROR; break; } // Timeout at 84 µs.

      if (PINx(PORT, PIN))
        break;

      // Bit ended.
      if (timer_counter_ >= 19 && timer_counter_ <= 31) // Between 20.80 and 33.60 µs.
        data_[byte_index_] &= ~(1 << bit_index_);
      else if (timer_counter_ >= 51 && timer_counter_ <= 78) // Between 56 and 84 µs.
        data_[byte_index_] |= (1 << bit_index_);
      else
      {
        //dht_humidity = 106;
        //dht_temperature = timer_counter_;
        state_ = ST_ERROR; // Erroneous bit.
        break;
      }

      if (++bit_index_ >= 8)
      {
        byte_index_++;
        state_ = ST_BYTE_TRANSFER_STARTING;
      }

      timer_counter_ = 0;
      TCNT0 = 0;
      break;

    default:
      break;
  }
}
