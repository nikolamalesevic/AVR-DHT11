 #include "DHT.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdbool.h>
#include "io_helper.h"
#include "time.h"

#define PORT  D
#define PIN   2

#define MS_TO_THRESHOLD(prescaler, ms) (uint8_t)((F_CPU * (ms)) / ((uint64_t)(prescaler) * 1000))

#define uS_TO_THRESHOLD(prescaler, us) (uint8_t)((F_CPU * (us)) / ((uint64_t)(prescaler) * 1000 * 1000))

/**
 * @brief Converts a duration to a timer output compare value.
 * @param   prescaler  A prescaler set on the timer.
 * @param   ms         A duration in milliseconds.
 * @return             A timer compare output value.
 */
#define THRESHOLD_TO_MS(prescaler, ms) ((F_CPU * (ms)) / ((uint64_t)(prescaler) * 1000))

/**
 * @brief Converts a duration to a timer output compare value.
 * @param   prescaler  A prescaler set on the timer.
 * @param   us         A duration in microseconds.
 * @return             A timer compare output value.
 */
#define THRESHOLD_TO_uS(prescaler, us) (THRESHOLD_TO_MS(prescaler, us) / 1000)

// /**
//  * @brief Converts a timer output compare value to a duration.
//  * @param   prescaler  A prescaler set on the timer.
//  * @param   timer      A timer compare output value.
//  * @return             A duration in milliseconds.
//  */
// #define TIMER_TO_MS(prescaler, timer) ((timer) * (prescaler) * 1000 / F_CPU)

// /**
//  * @brief Converts a timer output compare value to a duration.
//  * @param   prescaler  A prescaler set on the timer.
//  * @param   timer      A timer compare output value.
//  * @return             A duration in milliseconds.
//  */
// #define TIMER_TO_uS(prescaler, timer) (TIMER_TO_MS(prescaler, timer) * 1000)

typedef enum
{
  ST_IDLE,
  ST_ERROR,
  ST_ERROR_CHECKSUM,
  ST_ERROR_LOW_RESPONSE_TOO_SHORT,
  ST_ERROR_HIGH_RESPONSE_TOO_SHORT,
  ST_BIT_CLEARED,
  ST_WAIT_FOR_LOW_RESPONSE,
  ST_WAIT_FOR_HIGH_RESPONSE,
  ST_WAIT_FOR_HIGH_RESPONSE_END,
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
  TCCR0A = (1 << WGM01); // Timer CTC mode.
  time_subscribe_s(&on_second);
}

bool process_data()
{
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < 4; ++i)
    checksum += data_[i];

  if (data_[4] != checksum)
  {
    dht_temperature = 1077.9;
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
  TCCR0B = (1 << CS02) | (1 << CS00); // Set F_CPU/1024 prescaler.
  // OCR0 = 144; // Wait for 20 ms.
  OCR0A = MS_TO_THRESHOLD(1024, 20); // Wait for 20 ms.
  TCNT0 = 0;
  TIFR0 |= (1 << OCF0A);
  TIMSK0 |= (1 << OCIE0A); // Enable CTC interrupt for the timer.
  //TIFR0 |= (1 << OCF0A);
  sei(); // Enable global interrupts.
}

void on_second()
{
  //if (time_s % 5 == 0)
    read_data();
}

ISR(TIMER0_COMPA_vect)
{
  timer_counter_++;

  switch (state_)
  {
    case ST_BIT_CLEARED:
      // Set pin as input.
      PORTx_ON(PORT, PIN);
      IO_DIRECTION_IN(PORT, PIN);

      // Wait for response to start.
      state_ = ST_WAIT_FOR_LOW_RESPONSE;
      timer_counter_ = 0;
      TCCR0B = (1 << CS01); // Set F_CPU/8 prescaler.
      TCNT0 = 0;
      OCR0A = 1; // 1.085 µS
      break;

    case ST_WAIT_FOR_LOW_RESPONSE:
      // Wait for pin to become unset (20 µs min, 40 µs max).
      //if (timer_counter_ < uS_TO_THRESHOLD(8, 20))
        //break;

      if (timer_counter_ >= uS_TO_THRESHOLD(8, 40)) { dht_humidity = 110; state_ = ST_ERROR; break; } // Timeout at 40 µs.

      if (!PINx(PORT, PIN))
      {
        dht_temperature = timer_counter_ + 0.1;
        // Response started. Wait for response to finish.
        state_ = ST_WAIT_FOR_HIGH_RESPONSE;
        //state_ = ST_ERROR;
        timer_counter_ = 0;
        TCNT0 = 0;
      }

      break;
    
    case ST_WAIT_FOR_HIGH_RESPONSE:
      // Wait for pin to become set (80 µs min, 100 µs max).
      if (timer_counter_ < uS_TO_THRESHOLD(8, 80))
        break;
      //{
        //if (PINx(PORT, PIN))
        //{
          //dht_humidity = uS_TO_TIMER(8, 60);
          //dht_temperature = uS_TO_TIMER(8, 100);
          //state_ = ST_ERROR_LOW_RESPONSE_TOO_SHORT;
        //}
//
        //break;
      //}

      if (timer_counter_ >= uS_TO_THRESHOLD(8, 100)) { dht_humidity = 121; state_ = ST_ERROR; break; } // Timeout at 100 µs.

      if (PINx(PORT, PIN))
      {
        dht_temperature = timer_counter_ + 0.2;
        // Low response completed. Wait for high response.
        state_ = ST_WAIT_FOR_HIGH_RESPONSE_END;
        timer_counter_ = 0;
        TCNT0 = 0;
      }

      break;

    case ST_WAIT_FOR_HIGH_RESPONSE_END:
      // Wait for pin to become set (80 µs min, 100 µs max).
      if (timer_counter_ < uS_TO_THRESHOLD(8, 80))
        break;
      //{
        //if (!PINx(PORT, PIN))
        //{
          //dht_humidity = 130;
          //state_ = ST_ERROR_HIGH_RESPONSE_TOO_SHORT;
        //}
//
        //break;
      //}

      if (timer_counter_ >= uS_TO_THRESHOLD(8, 100)) { dht_humidity = 131; state_ = ST_ERROR; break; } // Timeout at 100 µs.

      if (!PINx(PORT, PIN))
      {
        dht_temperature = timer_counter_ + 0.3;
        // Low response completed. Wait for high response.
        state_ = ST_BYTE_TRANSFER_STARTING;
        timer_counter_ = 0;
        TCNT0 = 0;
        byte_index_ = 0;
      }

      break;

    /** Single byte processing. */

    case ST_BYTE_TRANSFER_STARTING:
      if (byte_index_ > 5)
      {
        if (process_data())
        {
          dht_temperature = byte_index_ + 0.4;
          state_ = ST_COMPLETED;
        }

        break;
      }

      bit_index_ = 0;
      state_ = ST_BIT_TRANSFER_STARTING;

    case ST_BIT_TRANSFER_STARTING:
      // Wait during low period for bit start (50 µs ideally).
      if (timer_counter_ < uS_TO_THRESHOLD(8, 30))
        break;
        
      if (timer_counter_ >= uS_TO_THRESHOLD(8, 70)) { dht_humidity = 160 + byte_index_ * 10 + bit_index_; state_ = ST_ERROR; break; } // Timeout at 60 µs.

      if (PINx(PORT, PIN))
      {
        dht_temperature = timer_counter_ + 0.5;
        // Data incoming.
        //state_ = ST_ERROR;
        state_ = ST_BIT_INCOMING;
        timer_counter_ = 0;
        TCNT0 = 0;
      }
      
      break;

    case ST_BIT_INCOMING:
      // Wait for bit end. 26 to 28 µs indicate low bit, 70 µs indicate high bit.
      if (timer_counter_ < uS_TO_THRESHOLD(8, 26))
        break;
      
      if (timer_counter_ >= uS_TO_THRESHOLD(8, 90)) { dht_humidity = 260 + byte_index_ * 10 + bit_index_; state_ = ST_ERROR; break; } // Timeout at 90 µs.

      dht_temperature = timer_counter_ + 0.6;

      if (PINx(PORT, PIN))
        break;

      dht_temperature = timer_counter_ + 0.7;

      // Bit ended.
      if (timer_counter_ >= uS_TO_THRESHOLD(8, 26) && timer_counter_ <= uS_TO_THRESHOLD(8, 28)) // Between 26 and 28 µs.
        data_[byte_index_] &= ~(1 << bit_index_);
      else if (timer_counter_ >= uS_TO_THRESHOLD(8, 56) && timer_counter_ <= uS_TO_THRESHOLD(8, 84)) // Between 56 and 84 µs.
        data_[byte_index_] |= (1 << bit_index_);
      else
      {
        dht_humidity = 260 + byte_index_ * 10 + bit_index_;
        dht_temperature = timer_counter_ + 0.8;
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
