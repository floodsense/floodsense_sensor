#include "sensorcfg.h"
#include "featherwing.h"

// File name
char name[] = "TSTLOG.TXT";
int SD_ERROR = 0;
// file system object
SdFat sd;
SdFile file;
SdFile myFile;
// create Serial stream
ArduinoOutStream cout(Serial);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
int calibration_const_sec = 5;
RTC_PCF8523 rtc;

void setup_featherWing(void) {

        //RTC init
        if (!rtc.begin()) {
                Serial.println("Couldn't find RTC");
                Serial.flush();
                abort();
        }

        if (!rtc.initialized() || rtc.lostPower()) {
                Serial.println("RTC is NOT initialized, let's set the time!");
                // When time needs to be set on a new device, or after a power loss, the
                // following line sets the RTC to the date & time this sketch was compiled
                rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
                // This line sets the RTC with an explicit date & time, for example to set
                // January 21, 2014 at 3am you would call:
                // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
                //
                // Note: allow 2 seconds after inserting battery or applying external power
                // without battery before calling adjust(). This gives the PCF8523's
                // crystal oscillator time to stabilize. If you call adjust() very quickly
                // after the RTC is powered, lostPower() may still return true.
        }

        // When time needs to be re-set on a previously configured device, the
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

}

String get_timestamp(void){
        DateTime now = rtc.now();
        String custom_timestamp_calibrated = String(now.year(), DEC) + String('-') + String(now.month(), DEC)+ String('-') + String(now.day(), DEC)+ String(' ') + String(now.hour(), DEC)+ String(':') + String(now.minute(), DEC) + String(':') + String(now.second(), DEC);
        return custom_timestamp_calibrated;
}

void writeToSDCard(String StringtobeWritten) {
        //add time stamp to every sd card write
        String timestamp = get_timestamp();
        StringtobeWritten = timestamp + String(',') +StringtobeWritten;
        char buffchar[StringtobeWritten.length() + 1];
        StringtobeWritten.toCharArray(buffchar, StringtobeWritten.length() + 1);
        digitalWrite(8, HIGH);
        if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
                // set the SD_ERROR flag high;
                SD_ERROR = 1;
                Serial.println("SD Initialization Failed.");
        }
        else
        {
                ofstream sdout(name, ios::out | ios::app);
                if (!sdout)
                {
                        Serial.println("SD Card Open Failed.");
                        SD_ERROR = 1;
                }
                else {
                        sdout << buffchar << endl;
                        // close the stream
                        sdout.close();
                        SD_ERROR = 0;
                }
        }
        digitalWrite(8, LOW);
}
uint8_t readkey(int m){
  digitalWrite(8, HIGH);
  Serial.begin(9600);
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {sd.initErrorHalt();}

  if(m==1){if (!myFile.open("one.txt", O_READ)) {
    sd.errorHalt("opening file one for read failed");
  }}
  if(m==2){if (!myFile.open("two.txt", O_READ)) {
    sd.errorHalt("opening file two for read failed");
  }}

  int data;
  char data1;
  uint8_t lo[8];
  uint8_t APPEUI11[8];
  int f=0;
  String temp;
  String temp1;
  int j=0;
  while ((data = myFile.read()) >= 0){data1=data;
  if(data == 44){int t = temp1.toInt();
  temp1="";
  lo[j]=t;
  APPEUI11[j]=t;
  j=j+1;
Serial.println(t);}
  else{temp1.concat(data1);}
  }
  myFile.close();
  digitalWrite(8, HIGH);
  return APPEUI11;
}
void writekey(int j,uint8_t ll[8]){
  digitalWrite(8, HIGH);
  if (!myFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
sd.errorHalt("opening test.txt for write failed");
}

  String la ;
  for(int i =0;i<8;i++){
    la.concat(String(ll[i]));
    la.concat(",");}

    myFile.rewind();
    myFile.println(la);
    myFile.close();
    Serial.println("done.");
   digitalWrite(8, LOW);
   Watchdog.enable(40);
}
