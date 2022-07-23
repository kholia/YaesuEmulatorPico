/**
  This source file is under General Public License version 3.

  Borrowed from uBITX code.

  https://github.com/afarhan/ubitxv6/blob/master/ubitx_cat.cpp
*/

#include <Wire.h>
#include <EEPROM.h>

void checkCAT();
unsigned char doingCAT = 0;
boolean txCAT = false;        // turned on if the transmitting due to a CAT command
char inTx = 0;                // it is set to 1 if in transmit mode (whatever the reason : cw, ptt or cat)
char isUSB = 0;
unsigned long frequency;

void active_delay(int delay_by) {
  unsigned long timeStart = millis();
  while (millis() - timeStart <= (unsigned long)delay_by) {
    delay(10);
    // Background Work
    checkCAT();
  }
}

void setFrequency(unsigned long f) {
  frequency = f;
}

void startTx() {
  digitalWrite(LED_BUILTIN, 1);
  inTx = 1;
}

void stopTx() {
  inTx = 0;
  digitalWrite(LED_BUILTIN, 0);
}

void setup()
{
  Serial.begin(19200);
  Serial.flush();

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  checkCAT();
}
