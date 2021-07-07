
#ifndef FEATHERWING_H
#define FEATHERWING_H

// SD CARD
#include <SdFat.h>

void writeToSDCard(String StringtobeWritten);
uint8_t readkey(String filename);
void writekey(String filename);
//RTC
#include "RTClib.h"

void setup_featherWing(void);
String get_timestamp(void);

#endif
