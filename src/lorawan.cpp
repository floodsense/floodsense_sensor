#include "lorawan.h"
#include "featherwing.h"
#include "functions.h"
#include "sleep.h"
#include "lmicpinmappings.h"
#include "sensorcfg.h"
#include "maxbotix.h"
#include <math.h>
#include <FlashStorage.h>

static osjob_t sendjob;           // lmic sendjob
unsigned int TX_INTERVAL;         // dutyCycle
int joinFailCounter = 0;          // counter for Join failure
unsigned char cfg_packet[7];      // CFG Uplink Packet
unsigned char lora_packet[5];     // Regular Uplink Packet
bool TX_COMPLETED = false;        // Set to false on start and after sleep; is set to true when an uplink is successful
bool UPDATE_CONFIG = true;        // Set to true at start and when there is a change in sensor cfg; used to send sensor cfg via uplink
uint16_t distance;
float measuredvbat;               // Measured Battery Level in mVolts
uint16_t batlevel;
unsigned int ERROR_FLAGS;

typedef struct {
        boolean init_;
        char useOrigApp;
} SelectAppEUI;

FlashStorage(USE_ORIG, SelectAppEUI);

FlashStorage(ORIG_APPEUI_0, uint8_t);
FlashStorage(ORIG_APPEUI_1, uint8_t);
FlashStorage(ORIG_APPEUI_2, uint8_t);
FlashStorage(ORIG_APPEUI_3, uint8_t);
FlashStorage(ORIG_APPEUI_4, uint8_t);
FlashStorage(ORIG_APPEUI_5, uint8_t);
FlashStorage(ORIG_APPEUI_6, uint8_t);
FlashStorage(ORIG_APPEUI_7, uint8_t);

FlashStorage(NEW_APPEUI_0, uint8_t);
FlashStorage(NEW_APPEUI_1, uint8_t);
FlashStorage(NEW_APPEUI_2, uint8_t);
FlashStorage(NEW_APPEUI_3, uint8_t);
FlashStorage(NEW_APPEUI_4, uint8_t);
FlashStorage(NEW_APPEUI_5, uint8_t);
FlashStorage(NEW_APPEUI_6, uint8_t);
FlashStorage(NEW_APPEUI_7, uint8_t);

SelectAppEUI selection;

void JoinFailCounter(void) {
        joinFailCounter = joinFailCounter + 1;
}

bool JoinFailed(void) {
        if (joinFailCounter == 5) {
                joinFailCounter = 0;
                return true;
        } else {
                return false;
        }
}

void JoinFailureDebug(void) {
        if(JoinFailed() && selection.useOrigApp == 'n' ) {
                // Use old AppEUI
                selection.useOrigApp = 'y';
                USE_ORIG.write(selection);
                // discard failed AppEUI
                NEW_APPEUI_0.write(0);
                NEW_APPEUI_1.write(0);
                NEW_APPEUI_2.write(0);
                NEW_APPEUI_3.write(0);
                NEW_APPEUI_4.write(0);
                NEW_APPEUI_5.write(0);
                NEW_APPEUI_6.write(0);
                NEW_APPEUI_7.write(0);
                // restart
                Watchdog.enable(1000);
        } else {
                Serial.println("Join falied. Check the default Keys loaded while flashing the MCU!");
        }
}


void os_getArtEui (u1_t* buf) {
        // choose AppEUI
        ChooseAppEUI();
        memcpy_P(buf, APPEUI, 8);
}

void os_getDevEui (u1_t* buf) {
        memcpy_P(buf, DEVEUI, 8);
}

void os_getDevKey (u1_t* buf) {
        memcpy_P(buf, APPKEY, 16);
}

void prepare_packet(void) {

        byte lowbyte, highbyte, lowbat, highbat;
        String packet_data;
        // Error
        ERROR_FLAGS = pow(2,0)* SD_ERROR;

        // Formatting payload
        if (UPDATE_CONFIG == true) {
                // Send Sensor Config via Uplink
                /*
                   CFG update uplink Format:
                 | Error Flag  | Sensor Mode | Sensor Sampling Rate | Sensor Number of Readings |
                 |    255 (FF) |    1 byte   |      2 bytes         |        1 bytes            |
                 */
                ERROR_FLAGS = 255;
                cfg_packet[0] = (unsigned char)ERROR_FLAGS;
                packet_data = String("CFG Update via Uplink");
                writeToSDCard(packet_data);
                byte lowduty = lowByte(TX_INTERVAL);
                byte highduty = highByte(TX_INTERVAL);
                cfg_packet[1] = (unsigned char)lowduty;
                cfg_packet[2] = (unsigned char)highduty;
                cfg_packet[3] = (unsigned char)sensorMode;
                lowbyte = lowByte(sensor_sampling_rate);
                highbyte = highByte(sensor_sampling_rate);
                cfg_packet[4] = (unsigned char)lowbyte;
                cfg_packet[5] = (unsigned char)highbyte;
                cfg_packet[6] = (unsigned char)sensor_numberOfReadings;
        }
        else {
                // Regular Uplink contains: Sensor Error Flags followed by Battery and then Sensor Data

                /* LoraWAN uplink packet format
                 | Error flags  | Battery Level | Ultrasonic reading  |
                 |   1 byte     |    2 bytes    |        2 bytes      |
                 |     Ultrasonic reading      |
                 |           2 bytes           |
                 |    high byte | low byte     |
                 |       Battery Level       |
                 |           2 bytes         |
                 |    high byte | low byte   |
                 |------------------------------------------------------------ Error Flags  ----------------------------------------------------------------|
                 |     bit 7                                                |  bit 6   |  bit 5  |  bit 4  |  bit 3  |  bit 2  |  bit 1  |      bit 0       |
                 |     Used only for CFG update (all other bits are high)   |          |         |         |         |         |         |   SD error flag  |
                 */



                // Maxbotix
                distance = read_sensor_using_modes(sensorMode, sensor_sampling_rate, sensor_numberOfReadings);
                Serial.print("Distance = ");
                Serial.print(distance);
                Serial.println(" mm");
                packet_data = String("Distance in mm is: ") + String(distance);
                writeToSDCard(packet_data);

                // Battery
                measuredvbat = analogRead(VBATPIN); //Float
                measuredvbat *= 2; // we divided by 2, so multiply back
                measuredvbat *= 3.3; // Multiply by 3.3V, our reference voltage
                measuredvbat /= 1024; // convert to voltage
                Serial.print("VBat: " ); Serial.println(measuredvbat);
                measuredvbat *= 1000; //make it milli volts to transmit
                Serial.print("VBat in mVolts: " ); Serial.println(measuredvbat);
                batlevel = measuredvbat; //Payload
                packet_data = String("Battery Level in mVolts is: ") + String(batlevel);
                writeToSDCard(packet_data);

                // Payload
                lowbat = lowByte(batlevel);
                highbat = highByte(batlevel);
                lora_packet[0] = (unsigned char)ERROR_FLAGS;
                packet_data = String("SD Error flag is: ") + String(SD_ERROR);
                writeToSDCard(packet_data);
                lora_packet[1] = (unsigned char)lowbat; //we're unsigned
                lora_packet[2] = (unsigned char)highbat;
                lowbyte = lowByte(distance);
                highbyte = highByte(distance);
                lora_packet[3] = (unsigned char)lowbyte;
                lora_packet[4] = (unsigned char)highbyte;
        }
}

void ModifyDutyCycle(void){
        unsigned long dutycycle = 0;
        for (int i = 1; i < 3; i++) {
                dutycycle =  (LMIC.frame[LMIC.dataBeg + i]) | ( dutycycle << 8*i);
        }
        if (dutycycle!= 0) {
                Serial.print("Current duty cycle is: ");
                Serial.println(TX_INTERVAL);
                String str_downlink = String("Current duty cycle is: ") + String(TX_INTERVAL);
                writeToSDCard(str_downlink);
                // Changing Duty Cycle
                TX_INTERVAL = dutycycle;
                Serial.print("Updated dutycycle is: ");
                Serial.println(TX_INTERVAL);
                str_downlink = String("Updated dutycycle is: ") + String(TX_INTERVAL);
                writeToSDCard(str_downlink);
        } else{
                Serial.println("Dutycycle is the same.");
        }
}

void ModifySensorMode(void){
        unsigned int sensorMode_ = 0;
        sensorMode_ =  (LMIC.frame[LMIC.dataBeg + 3]) | ( sensorMode_ );
        if (sensorMode_!= 0 && sensorMode<=3 ) {
                Serial.print("Current sensorMode is: ");
                Serial.println(sensorMode);
                String str_downlink = String("Current sensorMode is: ") + String(sensorMode);
                writeToSDCard(str_downlink);
                // Changing Sensor Mode
                sensorMode = sensorMode_;
                Serial.print("Updated sensorMode is: ");
                Serial.println(sensorMode);
                str_downlink = String("Updated sensorMode is: ") + String(sensorMode);
                writeToSDCard(str_downlink);
        } else{
                Serial.println("Sensor Mode is the same.");
        }
}

void ModifySamplingRate(void){
        unsigned int sampling_rate = 0;
        sampling_rate =  (LMIC.frame[LMIC.dataBeg + 4]) | ( sampling_rate );
        sampling_rate =  (LMIC.frame[LMIC.dataBeg + 5]) | ( sampling_rate << 8);
        if (sampling_rate!= 0 ) {
                Serial.print("Current sensor sampling rate is: ");
                Serial.println(sensor_sampling_rate);
                String str_downlink = String("Current sensor sampling rate is: ") + String(sensor_sampling_rate);
                writeToSDCard(str_downlink);
                // Changing Sensor Mode
                sensor_sampling_rate = sampling_rate;
                Serial.print("Updated sensor sampling rate is: ");
                Serial.println(sensor_sampling_rate);
                str_downlink = String("Updated sensor sampling rate is: ") + String(sensor_sampling_rate);
                writeToSDCard(str_downlink);
        } else{
                Serial.println("Sensor sampling rate is the same.");
        }
}

void ModifyNumberOfSamples(void){
        unsigned int numb_readings = 0;
        numb_readings =  (LMIC.frame[LMIC.dataBeg + 6]) | ( numb_readings );
        if (numb_readings!= 0 && numb_readings<20) {
                Serial.print("Current number of readings per measurement: ");
                Serial.println(sensor_numberOfReadings);
                String str_downlink = String("Current number of readings per measurement: ") + String(sensor_numberOfReadings);
                writeToSDCard(str_downlink);
                // Changing Sensor Mode
                sensor_numberOfReadings = numb_readings;
                Serial.print("Updated number of readings per measurement: ");
                Serial.println(sensor_numberOfReadings);
                str_downlink = String("Updated number of readings per measurement: ") + String(sensor_numberOfReadings);
                writeToSDCard(str_downlink);
        } else{
                Serial.println("Sensor number of readings per measurement is the same.");
        }
}

void ModifySensorSettings(void){
        switch(LMIC.dataLen) {
        case 3:
                ModifyDutyCycle();
                ModifySensorMode();
                break;
        case 4:
                ModifyDutyCycle();
                ModifySensorMode();
                break;
        case 6:
                ModifyDutyCycle();
                ModifySensorMode();
                ModifySamplingRate();
                break;
        case 7:
                ModifyDutyCycle();
                ModifySensorMode();
                ModifySamplingRate();
                ModifyNumberOfSamples();
                break;
        default:
                Serial.println("Invalid Sensor settings received.");
                break;
        }
}

void ModifyAppEUI(void){
        NEW_APPEUI_0.write(LMIC.frame[LMIC.dataBeg + 1]);
        NEW_APPEUI_1.write(LMIC.frame[LMIC.dataBeg + 2]);
        NEW_APPEUI_2.write(LMIC.frame[LMIC.dataBeg + 3]);
        NEW_APPEUI_3.write(LMIC.frame[LMIC.dataBeg + 4]);
        NEW_APPEUI_4.write(LMIC.frame[LMIC.dataBeg + 5]);
        NEW_APPEUI_5.write(LMIC.frame[LMIC.dataBeg + 6]);
        NEW_APPEUI_6.write(LMIC.frame[LMIC.dataBeg + 7]);
        NEW_APPEUI_7.write(LMIC.frame[LMIC.dataBeg + 8]);
        selection.useOrigApp = 'n'; // Use new AppEUI on restart
        USE_ORIG.write(selection); // Save the choice to Flash Memory
        Watchdog.enable(1000);   // Restart to start OTAA again
}

void process_received_downlink(void) {
        /*
         * Downlink Packet format:
         *    |   oper     | Duty Cycle in seconds  | Sensor Mode | Sampling Rate  |  Number of readings per measurement  |
         *    |  1 byte    |       2 bytes          |    1 byte   |    2 bytes     |        1 byte                        |
         */
        // set UPDATE_CONFIG to true
        UPDATE_CONFIG = true;
        String str_downlink = String("Processing received downlink....");
        writeToSDCard(str_downlink);
        uint8_t oper = LMIC.frame[LMIC.dataBeg];

        switch(oper) {
        case 0x4D:                             /* Change sensor Mode*/
                ModifySensorSettings();
                break;
        case 0x41:                             /* Change APPEUI*/
                ModifyAppEUI();
                break;
        default:
                Serial.println("Invalid downlink format!");
                break;
        }
}

void do_send(osjob_t* j) {
        // Check if there is not a current TX/RX job running
        if (LMIC.opmode & OP_TXRXPEND) {
                Serial.println(F("OP_TXRXPEND, not sending"));
                String do_sendstr1 = "OP_TXRXPEND, not sending";
                writeToSDCard(do_sendstr1);
        } else {
                // Prepare upstream data transmission at the next possible time.
                int lmic_tx_retVAL;
                if (UPDATE_CONFIG == true) {
                        lmic_tx_retVAL = LMIC_setTxData2(1, cfg_packet, sizeof(cfg_packet), 0);
                } else {
                        lmic_tx_retVAL = LMIC_setTxData2(1, lora_packet, sizeof(lora_packet), 0);
                }
                String do_sendstr;
                if (lmic_tx_retVAL == 0) {
                        Serial.println(F("Packet queued and lmic_tx_retVAL is 0."));
                } else {
                        do_sendstr = String ("Something is wrong: ") + String("Error number: ") + String(lmic_tx_retVAL);
                        Serial.println(do_sendstr);
                        writeToSDCard(do_sendstr);
                }
        }
        // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
        Serial.print(os_getTime());
        Serial.print(": ");
        String event_ev;
        switch (ev) {
        case EV_SCAN_TIMEOUT:
                Serial.println(F("EV_SCAN_TIMEOUT"));
                event_ev = String(event_ev + "EV_SCAN_TIMEOUT");
                writeToSDCard(event_ev);
                break;
        case EV_BEACON_FOUND:
                Serial.println(F("EV_BEACON_FOUND"));
                event_ev = String(event_ev + "EV_BEACON_FOUND");
                writeToSDCard(event_ev);
                break;
        case EV_BEACON_MISSED:
                Serial.println(F("EV_BEACON_MISSED"));
                event_ev = String(event_ev + "EV_BEACON_MISSED");
                writeToSDCard(event_ev);
        case EV_BEACON_TRACKED:
                Serial.println(F("EV_BEACON_TRACKED"));
                event_ev = String(event_ev + "EV_BEACON_TRACKED" );
                writeToSDCard(event_ev);
                break;
        case EV_JOINING:
                Serial.println(F("EV_JOINING"));
                event_ev = String(event_ev + "EV_JOINING");
                writeToSDCard(event_ev);
                break;
        case EV_JOINED:
                Serial.println(F("EV_JOINED"));
                event_ev = String(event_ev + "EV_JOINED" );
                writeToSDCard(event_ev);
                {
                        u4_t netid = 0;
                        devaddr_t devaddr = 0;
                        u1_t nwkKey[16];
                        u1_t artKey[16];
                        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
                        Serial.print("netid: ");
                        Serial.println(netid, DEC);
                        Serial.print("devaddr: ");
                        Serial.println(devaddr, HEX);
                        Serial.print("AppSKey: ");
                        for (size_t i = 0; i < sizeof(artKey); ++i) {
                                if (i != 0)
                                        Serial.print("-");
                                printHex2(artKey[i]);
                        }
                        Serial.println("");
                        Serial.print("NwkSKey: ");
                        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
                                if (i != 0)
                                        Serial.print("-");
                                printHex2(nwkKey[i]);
                        }
                        Serial.println();
                }
                // Disable link check validation (automatically enabled
                // during join, but because slow data rates change max TX
                // size, we don't use it in this example.
                LMIC_setLinkCheckMode(0);
                break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
                Serial.println(F("EV_JOIN_FAILED"));
                event_ev = String(event_ev + "EV_JOIN_FAILED");
                writeToSDCard(event_ev);
                JoinFailCounter();
                if(JoinFailed()) {
                        JoinFailureDebug();
                }
                break;
        case EV_REJOIN_FAILED:
                Serial.println(F("EV_REJOIN_FAILED"));
                event_ev = String(event_ev + "EV_REJOIN_FAILED" );
                writeToSDCard(event_ev);
                break;
                break;
        case EV_TXCOMPLETE:
                Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
                Serial.println("");
                Serial.println("");
                event_ev = String(event_ev + "EV_TXCOMPLETE (includes waiting for RX windows)");
                writeToSDCard(event_ev);
                if (LMIC.txrxFlags & TXRX_ACK)
                        Serial.println(F("Received ack"));
                event_ev = String("Received ack");
                writeToSDCard(event_ev);
                UPDATE_CONFIG = false;
                if (LMIC.dataLen) {
                        Serial.print(F("Received "));
                        Serial.print(LMIC.dataLen);
                        Serial.print(" bytes of payload: 0x");
                        for (int i = 0; i < LMIC.dataLen; i++) {
                                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                                        Serial.print(F("0"));
                                }
                                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                        }
                        Serial.println();
                        process_received_downlink();
                }
                TX_COMPLETED = true;
                break;
        case EV_LOST_TSYNC:
                Serial.println(F("EV_LOST_TSYNC"));
                event_ev = String(event_ev + "EV_LOST_TSYNC" );
                writeToSDCard(event_ev);
                break;
        case EV_RESET:
                Serial.println(F("EV_RESET"));
                event_ev = String(event_ev + "EV_RESET");
                writeToSDCard(event_ev);
                break;
        case EV_RXCOMPLETE:
                // data received in ping slot
                Serial.println(F("EV_RXCOMPLETE"));
                event_ev = String(event_ev + "EV_RXCOMPLETE");
                writeToSDCard(event_ev);
                break;
        case EV_LINK_DEAD:
                Serial.println(F("EV_LINK_DEAD"));
                event_ev = String(event_ev + "EV_LINK_DEAD");
                writeToSDCard(event_ev);
                break;
        case EV_LINK_ALIVE:
                Serial.println(F("EV_LINK_ALIVE"));
                event_ev = String(event_ev + "EV_LINK_ALIVE");
                writeToSDCard(event_ev);
                break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
                Serial.println(F("EV_TXSTART"));
                event_ev = String(event_ev + "EV_TXSTART");
                writeToSDCard(event_ev);
                break;
        case EV_TXCANCELED:
                Serial.println(F("EV_TXCANCELED"));
                event_ev = String(event_ev + "EV_TXCANCELED");
                writeToSDCard(event_ev);
                break;
        case EV_RXSTART:
                /* do not print anything -- it wrecks timing */
                break;
        case EV_JOIN_TXCOMPLETE:
                Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
                event_ev = String(event_ev + "EV_JOIN_TXCOMPLETE: no JoinAccept" );
                writeToSDCard(event_ev);
                break;
        default:
                Serial.print(F("Unknown event: "));
                event_ev = String(event_ev + "Unknown event");
                Serial.println((unsigned) ev);
                writeToSDCard(event_ev);
                break;
        }
}

void InitStoreAPPEUI(void){

        Serial.println("First time setup, Storing the original AppEUI...");
        // set useOrigApp to 'y'
        selection.useOrigApp = 'y';
        selection.init_ = true;
        USE_ORIG.write(selection);
        // Save APPEUI in the Flash Memory
        ORIG_APPEUI_0.write(APPEUI[0]);
        ORIG_APPEUI_1.write(APPEUI[1]);
        ORIG_APPEUI_2.write(APPEUI[2]);
        ORIG_APPEUI_3.write(APPEUI[3]);
        ORIG_APPEUI_4.write(APPEUI[4]);
        ORIG_APPEUI_5.write(APPEUI[5]);
        ORIG_APPEUI_6.write(APPEUI[6]);
        ORIG_APPEUI_7.write(APPEUI[7]);
        Serial.println("Original AppEUI saved in the Flash Memory.");
}

void LoadNewAppEUI(void){
        if ( NEW_APPEUI_0.read() != 0 && NEW_APPEUI_1.read() != 0 && NEW_APPEUI_2.read() != 0 && NEW_APPEUI_3.read() != 0 && NEW_APPEUI_4.read() != 0 && NEW_APPEUI_5.read() != 0 && NEW_APPEUI_6.read() != 0 && NEW_APPEUI_7.read() != 0) {
                // load new keys
                Serial.println("New AppEUI found");
                APPEUI[0] = NEW_APPEUI_0.read();
                APPEUI[1] = NEW_APPEUI_1.read();
                APPEUI[2] = NEW_APPEUI_2.read();
                APPEUI[3] = NEW_APPEUI_3.read();
                APPEUI[4] = NEW_APPEUI_4.read();
                APPEUI[5] = NEW_APPEUI_5.read();
                APPEUI[6] = NEW_APPEUI_6.read();
                APPEUI[7] = NEW_APPEUI_7.read();
                Serial.println("New AppEUI loaded.");
        } else {
                Serial.println("Key is zeroes and not valid. Using the original AppEUI");
                // key is zeroes and not valid
                selection.useOrigApp = 'y';
                USE_ORIG.write(selection);
        }
}

void lmicsetup(unsigned int packet_interval = 300) {
        Serial.println(F("Setting up LoraWAN..."));
        digitalWrite(13, HIGH);
        // LMIC init
        os_init();              // Reset the MAC state. Session and pending data transfers will be discarded.
        LMIC_reset();
        //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
        // Disable link check validation (automatically enabled)
        LMIC_setLinkCheckMode(0);
        LMIC_setDrTxpow(DR_SF7, 14);
        LMIC_selectSubBand(1);
        digitalWrite(13, LOW);
        Serial.println("Setup Ready!");
        TX_INTERVAL = packet_interval;
        Serial.println("Starting first job in setup");
        prepare_packet();
        selection = USE_ORIG.read();
        if (!selection.init_) {
                // First time setup...
                InitStoreAPPEUI();
        }
        if (selection.useOrigApp == 'n') {
                LoadNewAppEUI();   // choose AppEUI
        }
        do_send(&sendjob);      // Start job (sending automatically starts OTAA too)
}

void lorawan_runloop_once() {
        os_runloop_once();
        //&& TX_COMPLETED == true
        //!(LMIC.opmode & OP_TXRXPEND)
        if ( !os_queryTimeCriticalJobs(ms2osticksRound(8000) ) && TX_COMPLETED == true   ) {
                TX_COMPLETED = false;
                // This means the previous TX is complete and also no Critical Jobs pending in LMIC
                Serial.println("About to go to deep sleep and no critical jobs");
                //delay(30000);
                gotodeepsleepnow(TX_INTERVAL);
                Serial.println("Im awake and TX_COMPLETED is set to false");
                while(LMIC.opmode & OP_TXRXPEND) {
                        os_runloop_once();
                }

                //Prepare a packet in relaxed setiing
                prepare_packet();
                os_setCallback(&sendjob, do_send);
        }
}
