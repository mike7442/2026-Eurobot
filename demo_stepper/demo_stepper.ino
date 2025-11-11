#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper(2048, 14, 27);

void setup() {
  Serial.begin(115200);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  stepper.setRunMode(FOLLOW_POS);
  digitalWrite(25, 0);
  digitalWrite(26, 0);
}

void loop() {
    stepper.setTarget(1000);
}
