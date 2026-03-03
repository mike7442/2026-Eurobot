#include <ESP32Servo.h>

const int servoPins[] = { 32, 33, 15, 2 };

Servo servos[4];
int servosTargetPos[4] = { 90, 90, 90, 90 };
int servosCurrentPos[4] = { 90, 90, 90, 90 };
uint32_t servosTimer[4] = { 0, 0, 0, 0 };
int sDelay = 10;
int servosDelay[4] = { sDelay, sDelay, sDelay, sDelay };

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(3);
  ESP32PWM::allocateTimer(4);

  for (int i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 500, 2500);
    servos[i].write(servosCurrentPos[i]);
  }
}

void handleUARTCommand() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    int comma = cmd.indexOf(',');
    if (comma == -1) return;

    int servoNum = cmd.substring(0, comma).toInt();
    int angle = cmd.substring(comma + 1).toInt();

    if (servoNum >= 0 && servoNum < 4 && angle >= 0 && angle <= 180) {
      servosTargetPos[servoNum] = angle;
      Serial.printf("S%d:%d\n", servoNum, angle);
    }
  }
}

void servoPosControl() {
  for (int i = 0; i < 4; i++) {
    if (millis() - servosTimer[i] > servosDelay[i]) {
      int delta = servosCurrentPos[i] == servosTargetPos[i] ? 0 : (servosCurrentPos[i] < servosTargetPos[i] ? 1 : -1);
      servosCurrentPos[i] += delta;
      servosTimer[i] = millis();
      servos[i].write(servosCurrentPos[i]);
    }
  }
}

void loop() {
  servoPosControl();
  handleUARTCommand();
}
