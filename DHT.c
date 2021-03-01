 #include "DHT.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "io_helper.h"
#include "time.h"

#define PORT  D
#define PIN   2

/**
 * @brief Converts a duration to a timer threshold value.
 * @param   prescaler  A prescaler set on the timer.
 * @param   ms         A duration in milliseconds.
 * @return             A timer threshold.
 */
#define MS_TO_THRESHOLD(prescaler, ms) ((uint8_t)((F_CPU * (ms)) / ((uint64_t)(prescaler) * 1000)))

/**
 * @brief Converts a duration to a timer threshold value.
 * @param   prescaler  A prescaler set on the timer.
 * @param   us         A duration in microseconds.
 * @return             A timer threshold.
 */
#define uS_TO_THRESHOLD(prescaler, us) ((uint8_t)((F_CPU * (us)) / ((uint64_t)(prescaler) * 1000 * 1000)))

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
  TCCR0A = (1 << WGM00); // Normal timer mode.
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
  // Disable interrupts during processing, timing is critical.
  cli();
  
  // Set pin as output and low.
  IO_DIRECTION_OUT(PORT, PIN);
  PORTx_OFF(PORT, PIN);

  // Wait for 20 ms.
  TCCR0B = (1 << CS02) | (1 << CS00); // Set F_CPU/1024 prescaler.
  TCNT0 = 0;
  while (TCNT0 < MS_TO_THRESHOLD(1024, 20));
  
  // Set pin as input.
  PORTx_ON(PORT, PIN);
  IO_DIRECTION_IN(PORT, PIN);
  
  // Wait for low (max 20 ms).
  TCNT0 = 0;
  
  while (PINx(PORT, PIN))
  {
    if (TCNT0 >= MS_TO_THRESHOLD(1024, 20))
    {
      sei();
      return;
    }
  }
  
  // Switch to µs measurement.
  TCCR0B = (1 << CS01); // Set F_CPU/8 prescaler.
  
  // DHT11 low response starting. Wait for high 80 µs (min 60 µs, max 100 µs).
  TCNT0 = 0;
  
  while (!PINx(PORT, PIN))
  {
    if (TCNT0 >= uS_TO_THRESHOLD(8, 100))
    {
      sei();
      return;
    }
  }
  
  if (TCNT0 < uS_TO_THRESHOLD(8, 60))
  {
      sei();
      return;
  }
  
  // DHT11 high response starting. Wait for low 80 µs (min 60 µs, max 100 µs).
  TCNT0 = 0;
  
  while (PINx(PORT, PIN))
  {
    if (TCNT0 >= uS_TO_THRESHOLD(8, 100))
    {
      sei();
      return;
    }
  }
  
  if (TCNT0 < uS_TO_THRESHOLD(8, 60))
  {
    sei();
    return;
  }
  
  uint8_t bytes[5] = { 0 };
    
  //for (uint8_t i = 0; i < 5; ++i)
    //bytes[i] = 0;
  
  // Data incoming. Read 80 bits.
  for (uint8_t i = 0; i < 40; ++i)
  {
    // Bit starting. Wait for high 50 µs (min 40 µs, max 60 µs).
    TCNT0 = 0;
      
    while (!PINx(PORT, PIN))
    {
      if (TCNT0 >= uS_TO_THRESHOLD(8, 60))
      {
        sei();
        return;
      }
    }
      
    if (TCNT0 < uS_TO_THRESHOLD(8, 40))
    {
      sei();
      return;
    }
    
    // 0 or 1? Wait for low for:
    //   - 26-28 µs = 0, or
    //   - 70 µs    = 1.
    // (min 24 µs, max 80 µs).
    TCNT0 = 0;
    
    while (PINx(PORT, PIN))
    {
      if (TCNT0 >= uS_TO_THRESHOLD(8, 80))
      {
        sei();
        return;
      }
    }
    
    if (TCNT0 < uS_TO_THRESHOLD(8, 22))
    {
      sei();
      return;
    }
    else if (TCNT0 < uS_TO_THRESHOLD(8, 30))
    {
      // Bit = 0.
    }    
    else if (TCNT0 < uS_TO_THRESHOLD(8, 60))
    {
      sei();
      return;
    }
    else
    {
      // Bit = 1.
      bytes[i / 8] |= (1 << (7 - (i % 8)));
    }
  }
  
  sei();
  
  // Verify checksum.
  uint16_t sum = 0;
  
  for (int i = 0; i < 4; ++i)
    sum += bytes[i];
    
  if (sum != bytes[4])
    return;
  
  // Calculate values.
  float value = bytes[0];
  value += (float)bytes[1] * 0.1;
  dht_humidity = value;
  value = bytes[2];
  value += (float)bytes[3] * 0.1;
  dht_temperature= value;
}

void on_second()
{
//  if (time_s % 5 == 0)
    read_data();
}
