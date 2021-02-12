#ifndef _DHT_h
#define _DHT_h

#include <stdint.h>

extern uint8_t dht_humidity;
extern uint8_t dht_temperature;

/**
 * Initializes DHT sensor.
 * The sensor needs at least 2000 ms to start up after initial power-up.
 */
void dht_init();

#endif