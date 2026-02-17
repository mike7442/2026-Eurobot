#include <GyverStepper.h>
#define ENA 5
GStepper<STEPPER2WIRE> stepper(2048, 16, 17);

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT);
  stepper.setRunMode(FOLLOW_POS);
  digitalWrite(ENA, 0);
}

void loop() {
  if (!stepper.tick()) {
    static bool dir;
    dir = !dir;
    stepper.setTarget(dir ? -1024 : 1024);
  }
}
