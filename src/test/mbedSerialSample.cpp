//Path of pins_arduino.h --> /System/Volumes/Data/Users/PCOLLA/Library/Arduino15/packages/arduino/hardware/mbed_rp2040/4.0.2/variants/RASPBERRY_PI_PICO

#include "Wire.h"

MbedI2C myWire(16, 17);
UART mySerial(12, 13);

void setup() {
  myWire.begin();
  mySerial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

}
