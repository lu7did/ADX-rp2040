#include "PluggableUSBAudio.h"

USBAudio audio(true, 44100, 2, 44100, 2);

static uint8_t buf[128];
char hi[256];
int i=0;

void setup() {
  for (int i = 0; i<sizeof(buf); i++) {
    buf[i] = 128 * sin(i);
  }
  Serial.begin(115200);
  Serial.flush();
  sprintf(hi,"Test UAC2\n");
}

void loop() {
  audio.write(buf, sizeof(buf));
  i++;
  if (i>10000) {
     i=0;
     Serial.println("Loop()");
  }
}
