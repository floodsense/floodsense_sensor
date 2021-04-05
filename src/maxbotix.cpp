#include "maxbotix.h"
#include "functions.h"
#include "sensorcfg.h"
#include "featherwing.h"

// Maxbotix Ultrasonic variables
uint16_t readings_arr[20] = {0};
size_t n = *(&readings_arr + 1) - readings_arr;

unsigned int sensorMode;
unsigned int sensor_sampling_rate;
unsigned int sensor_numberOfReadings;

void setup_maxbotix(unsigned int mode=2, unsigned int sampling_rate=250 , unsigned int numberOfReadings=5) {
  String setup_mb;
  Serial.println("Setting up Maxbotix .... ");
  writeToSDCard(String("Setting up Maxbotix .... "));
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  Serial1.begin(9600);
  Serial.println("Sensor Settings:");
  writeToSDCard(String("Sensor Settings:"));
  sensorMode = mode;
  Serial.print("    Sensor mode: ");
  Serial.println(sensorMode);
  writeToSDCard(String("    Sensor mode: "+ sensorMode));
  sensor_sampling_rate = sampling_rate;
  Serial.print("    Sensor sampling rate: ");
  Serial.println(sensor_sampling_rate);
  writeToSDCard(String("    Sensor sampling rate: "+ sensor_sampling_rate));
  sensor_numberOfReadings = numberOfReadings;
  Serial.print("    Number of readings per measurement: ");
  Serial.println(sensor_numberOfReadings);
  writeToSDCard(String("    Number of readings per measurement: "+ sensor_numberOfReadings));
}

uint16_t sensor_singleread(void) {
  uint16_t distance;
  char buffer_RTT[6] = {};
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(20);            //Pin 4 ned to be pulled High for a minimum of 20 microseconds.
  digitalWrite(triggerPin, LOW);
  delay(150);
  if (Serial1.available() > 0) {
    do {
      if (Serial1.read() == 'R') {
        for (int i = 0; i < 5; i++) {
          buffer_RTT[i] = Serial1.read();
        }
      }
    } while (buffer_RTT == NULL);
    distance = (buffer_RTT[0] - '0') * 1000 + (buffer_RTT[1] - '0') * 100 + (buffer_RTT[2] - '0') * 10 + (buffer_RTT[3] - '0');
    Serial.print("distance: "); Serial.print(distance); Serial.println(" mm");
  }
  return distance;
}


uint16_t read_sensor_using_modes(unsigned int sensorMode, unsigned int sensor_sampling_rate=20, unsigned int sensor_numberOfReadings=5)
{
  String readings_array_string = "";
  uint16_t distance = 0;
  sensor_singleread();  // Ignore First read; First read in serial mode is incorrect
  Serial.println("Reading sensor using modes, entering measurements into an array...");
  writeToSDCard("Reading sensor using modes, entering measurements into an array...");
  for(int i=0;i<sensor_numberOfReadings;i++){
    readings_arr[i] = sensor_singleread();
    delay(sensor_sampling_rate);
  }
  Serial.println("Printing the array...");
  writeToSDCard("Printing the array...");
  for(int i=0; i<n; i++){
    Serial.print(readings_arr[i]);Serial.print(" ");
    readings_array_string = String(readings_array_string + readings_arr[i] + " ");
  }
  Serial.println("");
  writeToSDCard(readings_array_string);
  readings_array_string = ""; //Cleaning readings array string
  sort(readings_arr, n);
  Serial.println("Printing the sorted array...");
  writeToSDCard("Printing the sorted array...");
  for(int i=0; i<n; i++){
    Serial.print(readings_arr[i]);Serial.print(" ");
    readings_array_string = String(readings_array_string + readings_arr[i] + " ");
  }
  Serial.println("");
  writeToSDCard(readings_array_string);

  switch (sensorMode) {
    case 1:
      // Mean
      distance = mean(readings_arr, n, sensor_numberOfReadings);
      Serial.print("Mean is: "); Serial.println(distance);
      readings_array_string = String("Mean is: " + distance);
      writeToSDCard(readings_array_string);
      break;
    case 2:
      // Median
      distance = median(readings_arr, n, sensor_numberOfReadings);
      Serial.print("Median is: "); Serial.println(distance);
      readings_array_string = String("Median is: " + distance);
      writeToSDCard(readings_array_string);
      break;
    case 3:
      // Mode
      distance = mode(readings_arr, n, sensor_numberOfReadings);
      Serial.print("Mode is: "); Serial.println(distance);
      readings_array_string = String("Mode is: " + distance);
      writeToSDCard(readings_array_string);
    default:
      // Single Pulse-In single reading
      distance = sensor_singleread();
      Serial.print("Default single reading is: "); Serial.println(distance);
      readings_array_string = String("Default single reading is: " + distance);
      writeToSDCard(readings_array_string);
  }
  Serial.println("Cleaning measurements array...");
  writeToSDCard(String("Cleaning measurements array..."));
  for (int i=0; i<n; i++) {
    readings_arr[i] = 0;
  }
  Serial.println("");
  Serial.println("Printing the cleaned array...");
  writeToSDCard(String("Printing the cleaned array..."));
  for(int i=0; i<n; i++){
    Serial.print(readings_arr[i]);Serial.print(" ");
    readings_array_string = String(readings_array_string + readings_arr[i] + " ");
  }
  Serial.println("");
  writeToSDCard(readings_array_string);
  readings_array_string = ""; //Cleaning readings array string

  return distance;
}
