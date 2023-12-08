#include "MKS_SERVO57.h"

MKS_SERVO57 stepper;

void setup() {
  Serial.begin(115200);
  while (!Serial.available())
    delay(10);
  Serial1.begin(38400, SERIAL_8N1, D9, D8, false, 5000);
  stepper.initialize(&Serial1);
}

void loop() {
  Serial.println(stepper.setTargetPosition(1, 1, 1600, 15, 10000));
  delay(2000);
  Serial.println(stepper.setTargetPosition(1, 0, 1600, 15, 10000));
  delay(2000);
}
