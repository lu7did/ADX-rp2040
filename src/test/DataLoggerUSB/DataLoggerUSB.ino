
// Simple logger with USB upload to PC
// Uses SingleFileDrive to export an onboard LittleFS file to the computer
// The PC can open/copy the file, and then the user can delete it to restart

// Released to the public domain,  2022 - Earle F. Philhower, III

#include "SingleFileDrive.h"
#include <LittleFS.h>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "hardware/watchdog.h"
#include "Wire.h"
#include <EEPROM.h>
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/sio.h"
#include <stdio.h>
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include <WiFi.h>
#include <Time.h>
#include <stdbool.h>
#include <WiFiUdp.h>
#include <si5351.h>
#include "SPI.h"
#include <FS.h>

bool safe=true;
int lower = 1;
int upper = 6;
int count = 10;

void myPlugCB(uint32_t data) {
    // Tell my app not to write to flash, we're connected
    safe=false;    
}

void myUnplugCB(uint32_t data) {
    // I can start writing to flash again
    safe=true;
}

void myDeleteCB(uint32_t data) {
    // Maybe LittleFS.remove("myfile.txt")?  or do nothing
    Serial.println("remove file");
}

void setup() {
    Serial.begin(115200);
    while(!Serial);
    Serial.flush();
    Serial.println("SingleFileDrive ready");
    
    LittleFS.begin();
    singleFileDrive.onPlug(myPlugCB);
    singleFileDrive.onUnplug(myUnplugCB);
    singleFileDrive.onDelete(myDeleteCB);
    singleFileDrive.begin("littlefsfile.csv", "Data Recorder.csv");
}

void loop() {

   // Take some measurements, delay, etc.
   delay(1000);
    if (safe) {
        noInterrupts();
        File f = LittleFS.open("littlefsfile.csv", "a");

        int data1 = (rand() % (upper - lower + 1)) + lower;
        int data2 = (rand() % (upper - lower + 1)) + lower;
        int data3 = (rand() % (upper - lower + 1)) + lower;
        
        f.printf("%d,%d,%d\n", data1, data2, data3);
        f.close();
        interrupts();
        Serial.println("Wrote file");
    } else {
        Serial.println("Unsafe to write");
        delay(1000);
    }

}
