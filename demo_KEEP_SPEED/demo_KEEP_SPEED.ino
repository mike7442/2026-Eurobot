#include <GyverStepper.h>

#define ENA 4
GStepper<STEPPER2WIRE> stepper(800, 18, 19);

uint32_t timer = 0;
float currentSpeed = -2000;       // Начальная скорость
float targetSpeed = 2000;         // Целевая скорость
const float step = 10.0f;         // Шаг изменения скорости (плавность)

void setup() {
  pinMode(ENA, OUTPUT);
  stepper.setRunMode(KEEP_SPEED);
  stepper.setAcceleration(600);
  stepper.setMaxSpeed(2000);
  digitalWrite(ENA, LOW);
}

void loop() {
  if (millis() - timer >= 50) {  // Обновление каждые 50 мс
    if (currentSpeed < targetSpeed) {
      currentSpeed += step;
      if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
    } else if (currentSpeed > targetSpeed) {
      currentSpeed -= step;
      if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
    }

    stepper.setSpeed(currentSpeed);

    // Переключаем цель, когда достигли
    if (abs(currentSpeed - targetSpeed) < step) {
      targetSpeed = (targetSpeed == 2000) ? -2000 : 2000;
    }

    timer = millis();
  }
  stepper.tick();
}
