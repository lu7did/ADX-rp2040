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
