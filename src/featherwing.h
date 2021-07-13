#ifndef FEATHERWING_H
#define FEATHERWING_H

// SD CARD
#include <SdFat.h>

void writeToSDCard(String StringtobeWritten);

//RTC
#include "RTClib.h"

void setup_featherWing(void);
String get_timestamp(void);

#endif
