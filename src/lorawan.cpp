#include "lorawan.h"
#include "featherwing.h"
#include "functions.h"
#include "sleep.h"
#include "lmicpinmappings.h"
#include "sensorcfg.h"
#include "maxbotix.h"
#include <math.h>

static osjob_t sendjob;

unsigned int TX_INTERVAL;

unsigned char cfg_packet[7];
unsigned char lora_packet[5];
bool TX_COMPLETED = false;
      // Set to false on start and after sleep; is set to true when an uplink is successful
bool UPDATE_CONFIG = true;
int count = 0;       // Set to true at start and when there is a change in sensor cfg; used to send sensor cfg via uplink
static   u1_t PROGMEM APPEUI11[8]= {0x33, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
static  u1_t PROGMEM DEVEUI11[8]= {0x33, 0xBB, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
static  u1_t PROGMEM APPKEY11[16] = {0xE8, 0xF3, 0x44, 0x42, 0xBB, 0xDA, 0x06, 0x6B, 0x15, 0x5B, 0x1C, 0x3D, 0xEC, 0x0E, 0xF0, 0x1F};
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI11, 8);
}

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI11, 8);
}

void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY11, 16);
}

void lmicsetup( unsigned int packet_interval = 300) {       //Future setup variables
  Serial.println(F("Setting up LoraWAN..."));

  digitalWrite(13, HIGH);
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // Disable link check validation (automatically enabled)
  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7, 14);
  LMIC_selectSubBand(1);
  digitalWrite(13, LOW);
  Serial.println("Setup Ready!");
  TX_INTERVAL = packet_interval;
  // Start job (sending automatically starts OTAA too)
  Serial.println("Starting first job in setup");
  prepare_packet();
  do_send(&sendjob);
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
    if (UPDATE_CONFIG == true){
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

      event_ev = String(event_ev + "EV_TXCOMPLETE (includes waiting for RX windows)");
      writeToSDCard(event_ev);

      count = count+1;
      Serial.println(count);
      Serial.println("");
      if (count>2) {
        Serial.println("About to go to deep sleep and no critical jobsjvdhkgvxvcjcbxb");
        while(!os_queryTimeCriticalJobs(ms2osticksRound(29000) ) && TX_COMPLETED == true )
        {os_runloop_once();}        
        static   u1_t PROGMEM APPEUI11[8]= {0x22, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
        static  u1_t PROGMEM DEVEUI11[8]= {0x22, 0xBB, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
        static  u1_t PROGMEM APPKEY11[16] = {0x95, 0x41, 0xEB, 0x2A, 0xAC, 0xCD, 0x9E, 0x9B, 0xE4, 0x2E, 0x2F, 0x9F, 0xFC, 0x5D, 0x16, 0x9D};
        os_init();
        LMIC_reset();
        prepare_packet();
        do_send(&sendjob);

        //Serial.println("Setup Ready!");
        /* code */
      }
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      event_ev = String("Received ack");
      writeToSDCard(event_ev);
      UPDATE_CONFIG = false;
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        event_ev = String("Received downlinkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk ");
        writeToSDCard(event_ev);
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

void update_TX_INTERVAL(unsigned long dutycycle){
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
}

void update_sensorMode(unsigned int sensorMode_){
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
}

void update_sampling_rate(unsigned int sampling_rate){
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
}

void update_no_of_readings(unsigned int numb_readings){
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
}

void process_received_downlink(void){

  /* Downlink Packet format:
  |Duty Cycle in seconds  | Sensor Mode     | Sampling Rate    |  Number of readings per measurement  |
  | 2 byte                |    1 byte       |    2 bytes       |        1 bytes                       |
  */
  // set UPDATE_CONFIG to true
  UPDATE_CONFIG = true;
  String str_downlink = String("Processing received downlink....");
  writeToSDCard(str_downlink);
  unsigned int downlink_payload_size = LMIC.dataLen;
  unsigned long dutycycle = 0;
  unsigned int sensorMode_ = 0;
  unsigned int sampling_rate = 0;
  unsigned int numb_readings = 0;

  switch(downlink_payload_size){

    case 1: case 2:
      for (int i = 0; i < downlink_payload_size; i++) {
        dutycycle =  (LMIC.frame[LMIC.dataBeg + i]) | ( dutycycle << 8*i);
      }
      if (dutycycle!= 0){
        update_TX_INTERVAL(dutycycle);
      } else{
        Serial.println("Dutycycle is the same.");
      }
      break;

    case 3:
      for (int i = 0; i < 2; i++) {
        dutycycle =  (LMIC.frame[LMIC.dataBeg + i]) | ( dutycycle << 8*i);
      }
      if (dutycycle!= 0){
        update_TX_INTERVAL(dutycycle);
      } else{
        Serial.println("Dutycycle is the same.");
      }

      sensorMode_ =  (LMIC.frame[LMIC.dataBeg + 2]) | ( sensorMode_ );
      if (sensorMode_!= 0 && sensorMode<=3 ){
        update_sensorMode(sensorMode_);
      } else{
        Serial.println("Sensor Mode is the same.");
      }
      break;

    case 5:
      for (int i = 0; i < 2; i++) {
        dutycycle =  (LMIC.frame[LMIC.dataBeg + i]) | ( dutycycle << 8*i);
      }
      if (dutycycle!= 0){
        update_TX_INTERVAL(dutycycle);
      } else{
        Serial.println("Dutycycle is the same.");
      }
      sensorMode_ =  (LMIC.frame[LMIC.dataBeg + 2]) | ( sensorMode_ );
      if (sensorMode_!= 0 && sensorMode<=3 ){
        update_sensorMode(sensorMode_);
      } else{
        Serial.println("Sensor Mode is the same.");
      }
      sampling_rate =  (LMIC.frame[LMIC.dataBeg + 3]) | ( sampling_rate );
      sampling_rate =  (LMIC.frame[LMIC.dataBeg + 4]) | ( sampling_rate << 8);
      if (sampling_rate!= 0 ){
        update_sampling_rate(sampling_rate);
      } else{
        Serial.println("Sensor sampling rate is the same.");
      }
      break;

    case 6:
      for (int i = 0; i < 2; i++) {
        dutycycle =  (LMIC.frame[LMIC.dataBeg + i]) | ( dutycycle << 8*i);
      }
      if (dutycycle!= 0){
        update_TX_INTERVAL(dutycycle);
      } else{
        Serial.println("Dutycycle is the same.");
      }
      sensorMode_ =  (LMIC.frame[LMIC.dataBeg + 2]) | ( sensorMode_ );
      if (sensorMode_!= 0 && sensorMode<=3 ){
        update_sensorMode(sensorMode_);
      } else{
        Serial.println("Sensor Mode is the same.");
      }
      sampling_rate =  (LMIC.frame[LMIC.dataBeg + 3]) | ( sampling_rate );
      sampling_rate =  (LMIC.frame[LMIC.dataBeg + 4]) | ( sampling_rate << 8);
      if (sampling_rate!= 0 ){
        update_sampling_rate(sampling_rate);
      } else{
        Serial.println("Sensor sampling rate is the same.");
      }
      numb_readings =  (LMIC.frame[LMIC.dataBeg + 5]) | ( numb_readings );
      if (numb_readings!= 0 && numb_readings<=20){
        update_no_of_readings(numb_readings);
      } else{
        Serial.println("Sensor number of readings per measurement is the same.");
      }
      break;
  }

}
void process_received_downlink11(void) {
  Serial.println("About to go to deep sleep and no critical jobsjvdhkgvxvcjcbxb");
  LMIC_reset();
  String wr;
  wr = String("Received downlinkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk11111111111111111 ");
  writeToSDCard(wr);
  static   u1_t PROGMEM APPEUI11[8]= {0x22, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
  static  u1_t PROGMEM DEVEUI11[8]= {0x22, 0xBB, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
  static  u1_t PROGMEM APPKEY11[16] = {0x95, 0x41, 0xEB, 0x2A, 0xAC, 0xCD, 0x9E, 0x9B, 0xE4, 0x2E, 0x2F, 0x9F, 0xFC, 0x5D, 0x16, 0x9D};

  prepare_packet();
  do_send(&sendjob);
  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF7, 14);
  LMIC_selectSubBand(1);
  digitalWrite(13, LOW);
  Serial.println("Setup Ready!");
}

uint16_t distance;
// Measured Battery Level in mVolts
float measuredvbat;
uint16_t batlevel;
unsigned int ERROR_FLAGS;

void prepare_packet(void) {

  byte lowbyte, highbyte, lowbat, highbat;
  String packet_data;
  // Error
  ERROR_FLAGS = pow(2,0)* SD_ERROR;

  // Formatting payload
  if (UPDATE_CONFIG == true){
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
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
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

void lorawan_runloop_once() {
  os_runloop_once();
  //&& TX_COMPLETED == true
  //!(LMIC.opmode & OP_TXRXPEND)
  if ( !os_queryTimeCriticalJobs(ms2osticksRound(29000) ) && TX_COMPLETED == true   ) {
    TX_COMPLETED = false;
    // This means the previous TX is complete and also no Critical Jobs pending in LMIC
    Serial.println("About to go to deep sleep and no critical jobs");
    delay(8000);
    //gotodeepsleepnow(8000);
    Serial.println("Im awake and TX_COMPLETED is set to false");
    while(LMIC.opmode & OP_TXRXPEND){
      //Serial.println("waiting for OP_TXRXPEND");
      os_runloop_once();
    }

     //Prepare a packet in relaxed setiing
    prepare_packet();
    os_setCallback(&sendjob, do_send);
  }
}
