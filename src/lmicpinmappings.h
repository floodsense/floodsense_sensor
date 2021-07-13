#ifndef LMICPINMAPPINGS_H
#define LMICPINMAPPINGS_H

#include "lorawan.h"

// Pin mapping
//
// Adafruit BSPs are not consistent -- m0 express defs ARDUINO_SAMD_FEATHER_M0,
// m0 defs ADAFRUIT_FEATHER_M0
//
#if defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0)
// Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
        .nss = 8,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 4,
        .dio = {3, 6, LMIC_UNUSED_PIN},
        .rxtx_rx_active = 0,
        .rssi_cal = 8,        // LBT cal for the Adafruit Feather M0 LoRa, in dB
        .spi_freq = 8000000,
};
#elif defined(ARDUINO_AVR_FEATHER32U4)
// Pin mapping for Adafruit Feather 32u4 LoRa, etc.
// Just like Feather M0 LoRa, but uses SPI at 1MHz; and that's only
// because MCCI doesn't have a test board; probably higher frequencies
// will work.
const lmic_pinmap lmic_pins = {
        .nss = 8,
        .rxtx = LMIC_UNUSED_PIN,
        .rst = 4,
        .dio = {7, 6, LMIC_UNUSED_PIN},
        .rxtx_rx_active = 0,
        .rssi_cal = 8,        // LBT cal for the Adafruit Feather 32U4 LoRa, in dB
        .spi_freq = 1000000,
};
#elif defined(ARDUINO_CATENA_4551)
// Pin mapping for Murata module / Catena 4551
const lmic_pinmap lmic_pins = {
        .nss = 7,
        .rxtx = 29,
        .rst = 8,
        .dio = {
                25, // DIO0 (IRQ) is D25
                26, // DIO1 is D26
                27, // DIO2 is D27
        },
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 8000000 // 8MHz
};
#else
# error "Unknown target"
#endif

#endif
