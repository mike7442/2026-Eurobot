#include <GyverStepper.h>
#define ENA 4
GStepper<STEPPER2WIRE> stepper(800, 18, 19);

void setup() {
  pinMode(ENA, OUTPUT);
  stepper.setRunMode(FOLLOW_POS);
  stepper.setAcceleration(600);
  stepper.setMaxSpeed(2000);
  digitalWrite(ENA, 0);
}

void loop() {
  if (!stepper.tick()) {
    static bool dir;
    dir = !dir;
    stepper.setTarget(dir ? -1024 : 1024);
  }
}
