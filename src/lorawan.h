#ifndef LORAWAN_H
#define LORAWAN_H

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Arduino.h>
#include "ttncredentials.h"


// This EUI must be in little-endian format
static u1_t PROGMEM APPEUI[8] = TTN_APPEUI ;
// This should also be in little endian format
static const u1_t PROGMEM DEVEUI[8] = TTN_DEVEUI ;
// This key should be in big endian format
static const u1_t PROGMEM APPKEY[16] = TTN_APPKEY ;

void lorawan_runloop_once(void);
void lmicsetup(unsigned int packet_interval);

#endif
