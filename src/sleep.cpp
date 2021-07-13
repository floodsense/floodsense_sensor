#include "sleep.h"
#include "sensorcfg.h"

void gotodeepsleepnow(unsigned int sleep_time) {
        int sleep_cycles = sleep_time / 8;
        Serial.println("About to sleep");
        for (int i = 0; i < sleep_cycles; i++) {
                Watchdog.sleep(8000);
        }
        Serial.println("Im Awake!");
}
