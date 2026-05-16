#include <Arduino.h>
#include <GyverStepper.h>
#include <ESP32Servo.h>

// ===================== ПИНЫ =====================

const int ROUTE_PIN = 21;
const int START_PIN = 2;

// Лидар
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN -1
#define LIDAR_BAUDRATE 115200

// До 6 серв
#define SERVO_UNUSED -1

const int SERVO_PINS[6] = {
  SERVO_UNUSED,
  SERVO_UNUSED,
  SERVO_UNUSED,
  SERVO_UNUSED,
  SERVO_UNUSED,
  SERVO_UNUSED
};

const int SERVO_COUNT = 6;

// Шаговые двигатели
GStepper<STEPPER2WIRE> stepper1(800, 19, 18, 4);
GStepper<STEPPER2WIRE> stepper2(800, 23, 22, 5);

// Сервы
Servo servos[SERVO_COUNT];

// ===================== СЕКТОРА ЛИДАРА =====================

#define NUM_SECTORS 12
#define SEC(n) (1 << (n))

#define SECTORS_NONE 0
#define SECTORS_ALL  0x0FFF

#define SECTORS_FRONT (SEC(7) | SEC(8) | SEC(9) | SEC(10) | SEC(11) | SEC(0))
#define SECTORS_BACK  (SEC(1) | SEC(2) | SEC(3) | SEC(4) | SEC(5) | SEC(6))

int sectorStatus[NUM_SECTORS] = { 0 };

// ===================== ПАРАМЕТРЫ ЛИДАРА =====================

static const uint8_t LIDAR_HEADER[] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t LIDAR_HEADER_LEN = 4;
static const uint8_t LIDAR_BODY_LEN = 32;

#define ALARM_DIST 400
#define WARNING_DIST 650
#define ALARM_HOLD_MS 300
#define SECTOR_OFFSET 1

unsigned long startTime = 0;
bool forcedStop = false;

static float sectorDistances[NUM_SECTORS] = { 0.0f };
static uint32_t sectorUpdateTime[NUM_SECTORS] = { 0 };
static uint32_t sectorAlarmUntil[NUM_SECTORS] = { 0 };

static const float NO_VALUE = 99999.0f;

// ===================== ТИПЫ ШАГОВ =====================

enum StepType {
  MOVE_STEPPERS,
  SERVO_MOVE,
  PAUSE_STEP,
  END_ROUTE
};

struct RouteStep {
  StepType type;

  long motor1Steps;
  long motor2Steps;

  int servoIndex;
  int servoAngle;

  unsigned long timeMs;

  uint16_t checkSectors;
};

// ===================== МАРШРУТЫ =====================

RouteStep route1[] = {
  { MOVE_STEPPERS, 10070, 10070, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 1000, -1000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 1400, 1400, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 1000, -1000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 8900, 8900, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -1500, -1500, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 1000, -1000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -2000, -2000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 5500, 5500, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -4000, -4000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -1000, 1000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 2500, 2500, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 0, 0, -1, 0, 0, SECTORS_NONE },
  { PAUSE_STEP, 0, 0, -1, 0, 1000, SECTORS_NONE },

  // Поворот — проверяем все сектора
  { MOVE_STEPPERS, -200, 200, -1, 0, 0, SECTORS_ALL },

  { END_ROUTE, 0, 0, -1, 0, 0, SECTORS_NONE }
};

RouteStep route2[] = {
  { MOVE_STEPPERS, 10050, 10050, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -1000, 1000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 1400, 1400, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -1000, 1000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 8900, 8900, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -1500, -1500, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -1000, 1000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -2000, -2000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 5500, 5500, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, -4000, -4000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 1000, -1000, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 2500, 2500, -1, 0, 0, SECTORS_ALL },
  { MOVE_STEPPERS, 0, 0, -1, 0, 0, SECTORS_NONE },
  { PAUSE_STEP, 0, 0, -1, 0, 1000, SECTORS_NONE },

  // Поворот — проверяем все сектора
  { MOVE_STEPPERS, -200, 200, -1, 0, 0, SECTORS_ALL },

  { END_ROUTE, 0, 0, -1, 0, 0, SECTORS_NONE }
};

// ===================== ПЕРЕМЕННЫЕ МАРШРУТА =====================

RouteStep *currentRoute;

int currentStepIndex = 0;

bool stepStarted = false;
bool motorsWereMoving = false;

unsigned long stepStartTime = 0;

int activeServoIndex = -1;

// ===================== ЛИДАР =====================

bool readBytesWithTimeout(HardwareSerial &ser, uint8_t *buffer, size_t length, uint32_t timeout_ms = 2) {
  uint32_t start = millis();
  size_t count = 0;

  while (count < length) {
    if (ser.available()) {
      buffer[count++] = ser.read();
    }

    if (millis() - start > timeout_ms) {
      return false;
    }
  }

  return true;
}

bool waitForHeader(HardwareSerial &ser) {
  uint8_t matchPos = 0;
  uint32_t start = millis();

  while (millis() - start < 1) {
    if (ser.available()) {
      uint8_t b = ser.read();

      if (b == LIDAR_HEADER[matchPos]) {
        matchPos++;

        if (matchPos == LIDAR_HEADER_LEN) {
          return true;
        }
      } else {
        matchPos = 0;
      }
    }
  }

  return false;
}

float decodeAngle(uint16_t rawAngle) {
  float angleDeg = (float)(rawAngle - 0xA000) / 64.0f;

  while (angleDeg < 0) angleDeg += 360.0f;
  while (angleDeg >= 360) angleDeg -= 360.0f;

  return angleDeg;
}

int angleToSector(float angleDeg) {
  float shifted = angleDeg + 15.0f;

  while (shifted < 0) shifted += 360.0f;
  while (shifted >= 360) shifted -= 360.0f;

  int sector = (int)(shifted / 30.0f) % NUM_SECTORS;
  sector = (sector + SECTOR_OFFSET) % NUM_SECTORS;

  return sector;
}

void updateSectorStatuses() {
  uint32_t now = millis();

  for (int s = 0; s < NUM_SECTORS; s++) {
    float dist = sectorDistances[s];

    if (dist == NO_VALUE) {
      sectorStatus[s] = 0;
    } else if (now < sectorAlarmUntil[s]) {
      sectorStatus[s] = 2;
    } else if (dist < WARNING_DIST) {
      sectorStatus[s] = 1;
    } else {
      sectorStatus[s] = 0;
    }
  }
}

bool parseAndProcessLidarPacket() {
  if (!waitForHeader(Serial1)) {
    updateSectorStatuses();
    return false;
  }

  uint8_t buffer[LIDAR_BODY_LEN];

  if (!readBytesWithTimeout(Serial1, buffer, LIDAR_BODY_LEN, 2)) {
    updateSectorStatuses();
    return false;
  }

  uint16_t startAngleTmp = buffer[2] | (buffer[3] << 8);
  float startAngleDeg = decodeAngle(startAngleTmp);

  uint8_t offset = 4;

  uint16_t distances[8];
  uint8_t intensities[8];

  for (int i = 0; i < 8; i++) {
    distances[i] = buffer[offset] | (buffer[offset + 1] << 8);
    intensities[i] = buffer[offset + 2];
    offset += 3;
  }

  uint16_t endAngleTmp = buffer[offset] | (buffer[offset + 1] << 8);
  float endAngleDeg = decodeAngle(endAngleTmp);

  if (endAngleDeg < startAngleDeg) {
    endAngleDeg += 360.0f;
  }

  float angleRange = endAngleDeg - startAngleDeg;
  float angleInc = angleRange / 8.0f;

  float tempSectorMin[NUM_SECTORS];

  for (int s = 0; s < NUM_SECTORS; s++) {
    tempSectorMin[s] = NO_VALUE;
  }

  for (int i = 0; i < 8; i++) {
    if (intensities[i] > 15) {
      float angle = startAngleDeg + i * angleInc;

      while (angle < 0) angle += 360.0f;
      while (angle >= 360) angle -= 360.0f;

      int sectorIndex = angleToSector(angle);
      float dist = (float)distances[i];

      if (dist < tempSectorMin[sectorIndex]) {
        tempSectorMin[sectorIndex] = dist;
      }
    }
  }

  uint32_t now = millis();

  for (int s = 0; s < NUM_SECTORS; s++) {
    if (tempSectorMin[s] != NO_VALUE) {
      sectorDistances[s] = tempSectorMin[s];
      sectorUpdateTime[s] = now;
    }
  }

  for (int s = 0; s < NUM_SECTORS; s++) {
    if ((now - sectorUpdateTime[s]) > 500) {
      sectorDistances[s] = NO_VALUE;
    }
  }

  for (int s = 0; s < NUM_SECTORS; s++) {
    float dist = sectorDistances[s];

    if (dist != NO_VALUE && dist < ALARM_DIST) {
      sectorAlarmUntil[s] = now + ALARM_HOLD_MS;
    }
  }

  updateSectorStatuses();
  return true;
}

bool isBlockedByLidar(uint16_t checkSectors) {
  if (checkSectors == SECTORS_NONE) {
    return false;
  }

  for (int s = 0; s < NUM_SECTORS; s++) {
    if (checkSectors & SEC(s)) {
      if (sectorStatus[s] == 2) {
        return true;
      }
    }
  }

  return false;
}

void printSectorStatuses() {
  Serial.print("SECTORS: ");

  for (int s = 0; s < NUM_SECTORS; s++) {
    Serial.print(sectorStatus[s]);

    if (s < NUM_SECTORS - 1) {
      Serial.print(",");
    }
  }

  Serial.println();
}

// ===================== МОТОРЫ =====================

void setupSteppers() {
  stepper1.setRunMode(FOLLOW_POS);
  stepper2.setRunMode(FOLLOW_POS);

  stepper1.setMaxSpeed(3000);
  stepper1.setAcceleration(1000);

  stepper2.setMaxSpeed(3000);
  stepper2.setAcceleration(1000);

  stepper1.autoPower(true);
  stepper2.autoPower(true);

  stepper1.invertEn(1);
  stepper2.invertEn(1);

  // stepper1.reverse(1);
  stepper2.reverse(1);

  stepper1.enable();
  stepper2.enable();
}

// ===================== СЕРВЫ =====================

bool isServoAvailable(int index) {
  if (index < 0 || index >= SERVO_COUNT) return false;
  if (SERVO_PINS[index] == SERVO_UNUSED) return false;
  return true;
}

void attachAndWriteServo(int index, int angle) {
  if (!isServoAvailable(index)) {
    Serial.print("SERVO ");
    Serial.print(index);
    Serial.println(" is unused. Skip.");
    return;
  }

  angle = constrain(angle, 0, 180);

  servos[index].attach(SERVO_PINS[index], 500, 2500);
  servos[index].write(angle);

  activeServoIndex = index;

  Serial.print("SERVO ");
  Serial.print(index);
  Serial.print(" on pin ");
  Serial.print(SERVO_PINS[index]);
  Serial.print(" -> ");
  Serial.println(angle);
}

void detachActiveServo() {
  if (isServoAvailable(activeServoIndex)) {
    servos[activeServoIndex].detach();

    Serial.print("SERVO ");
    Serial.print(activeServoIndex);
    Serial.println(" DETACH");
  }

  activeServoIndex = -1;
}

// ===================== ЗАПУСК ШАГА =====================

void startRouteStep(RouteStep step) {
  stepStartTime = millis();

  if (step.type == MOVE_STEPPERS) {
    motorsWereMoving = false;

    Serial.print("MOVE: ");
    Serial.print(step.motor1Steps);
    Serial.print(" / ");
    Serial.println(step.motor2Steps);

    stepper1.setTarget(step.motor1Steps, RELATIVE);
    stepper2.setTarget(step.motor2Steps, RELATIVE);
  }

  else if (step.type == SERVO_MOVE) {
    attachAndWriteServo(step.servoIndex, step.servoAngle);
  }

  else if (step.type == PAUSE_STEP) {
    Serial.print("PAUSE: ");
    Serial.print(step.timeMs);
    Serial.println(" ms");
  }

  stepStarted = true;
}

// ===================== ПРОВЕРКА ЗАВЕРШЕНИЯ ШАГА =====================

bool isRouteStepFinished(RouteStep step, byte stepper1Move, byte stepper2Move) {
  if (step.type == MOVE_STEPPERS) {
    if (stepper1Move != 0 || stepper2Move != 0) {
      motorsWereMoving = true;
    }

    return motorsWereMoving && stepper1Move == 0 && stepper2Move == 0;
  }

  if (step.type == SERVO_MOVE) {
    if (millis() - stepStartTime >= step.timeMs) {
      detachActiveServo();
      return true;
    }

    return false;
  }

  if (step.type == PAUSE_STEP) {
    return millis() - stepStartTime >= step.timeMs;
  }

  return false;
}

// ===================== SETUP =====================

void setup() {
  unsigned long startTime = millis(); // переменная для отсчёта 100 сек со старта

  Serial.begin(115200);

  pinMode(ROUTE_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);

  Serial1.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  setupSteppers();

  Serial.println("Robot route program with lidar safety");

  Serial.println("Servo configuration:");
  for (int i = 0; i < SERVO_COUNT; i++) {
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(": ");

    if (SERVO_PINS[i] == SERVO_UNUSED) {
      Serial.println("UNUSED");
    } else {
      Serial.print("GPIO ");
      Serial.println(SERVO_PINS[i]);
    }
  }

  if (digitalRead(ROUTE_PIN) == LOW) {
    currentRoute = route2;
    Serial.println("Selected route: 2");
  } else {
    currentRoute = route1;
    Serial.println("Selected route: 1");
  }

  Serial.println("Waiting START_PIN press...");

  while (digitalRead(START_PIN) == HIGH) {
    parseAndProcessLidarPacket();
    delay(1);
  }

  Serial.println("START!");
}

// ===================== LOOP =====================

void loop() {
    if (millis() - startTime >= 100000) {
    Serial.println("100 SECONDS — STOP!");

    stepper1.disable();
    stepper2.disable();

    for (int i = 0; i < SERVO_COUNT; i++) {
      servos[i].detach();
    }

    while (true) {
      delay(100);
    }
  }

  RouteStep currentStep = currentRoute[currentStepIndex];

  if (currentStep.type == END_ROUTE) {
    Serial.println("ROUTE FINISHED");

    while (true) {
      parseAndProcessLidarPacket();
      delay(10);
    }
  }

  bool blocked = isBlockedByLidar(currentStep.checkSectors);

  static bool wasBlocked = false;

  byte stepper1Move = 0;
  byte stepper2Move = 0;

  // ВАЖНО:
  // tick() вызываем ДО чтения лидара.
  // Так шаговые двигатели получают импульсы чаще и не тормозят.
  //
  // Если лидар видит препятствие — tick() не вызываем,
  // и шаговые двигатели фактически замирают на паузе.
  if (!blocked) {
    stepper1Move = stepper1.tick();
    stepper2Move = stepper2.tick();
  }

  // Лидар читаем после tick(), чтобы он не душил скорость шаговиков
  parseAndProcessLidarPacket();

  if (blocked) {
    if (!wasBlocked) {
      Serial.println("LIDAR BLOCKED. Waiting...");
      printSectorStatuses();
      wasBlocked = true;
    }

    delay(1);
    return;
  }

  if (wasBlocked) {
    Serial.println("LIDAR CLEAR. Continue.");
    printSectorStatuses();
    wasBlocked = false;
  }

  if (!stepStarted) {
    startRouteStep(currentStep);
  }

  if (isRouteStepFinished(currentStep, stepper1Move, stepper2Move)) {
    Serial.print("Step finished: ");
    Serial.println(currentStepIndex);

    currentStepIndex++;
    stepStarted = false;
    motorsWereMoving = false;

    delay(10);
  }
}
