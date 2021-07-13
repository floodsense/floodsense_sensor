#include "functions.h"

void printHex2(unsigned v) {
        v &= 0xff;
        if (v < 16)
                Serial.print('0');
        Serial.print(v, HEX);
}

void swap(uint16_t  *p, uint16_t  *q) {
        int t;

        t = *p;
        *p = *q;
        *q = t;
}

void sort(uint16_t a[], size_t n) {
        int i, j, temp;

        for (i = 0; i < n - 1; i++) {
                for (j = 0; j < n - i - 1; j++) {
                        if (a[j] < a[j + 1])
                                swap(&a[j], &a[j + 1]);
                }
        }
}

uint16_t mean(uint16_t readings_arr[], size_t n, unsigned int sensor_numberOfReadings){
        uint16_t readings_sum = 0;
        for (int i=0; i<sensor_numberOfReadings; i++) {
                readings_sum = readings_sum + readings_arr[i];
        }
        return readings_sum/sensor_numberOfReadings;
}

uint16_t median(uint16_t readings_arr[], size_t n, unsigned int sensor_numberOfReadings){
        if (sensor_numberOfReadings%2 != 0) {
                return readings_arr[(sensor_numberOfReadings + 1) / 2 - 1];
        }
        else {
                return (readings_arr[sensor_numberOfReadings/2 -1]+ readings_arr[sensor_numberOfReadings/2])/2;
        }
}

uint16_t mode(uint16_t readings_arr[], size_t n, unsigned int sensor_numberOfReadings){
        int counter = 1;
        int max = 0;
        uint16_t mode_ = readings_arr[0];
        for (int i = 0; i < sensor_numberOfReadings - 1; i++) {
                if ( readings_arr[i] == readings_arr[i+1] ) {
                        counter++;
                        if ( counter > max ) {
                                max = counter;
                                mode_ = readings_arr[i];
                        }
                } else
                        counter = 1; // reset counter.
        }
        return mode_;
}
