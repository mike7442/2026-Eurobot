#include <GyverStepper.h>
#include <Arduino.h>

/********************************************************
 *  КОНСТАНТЫ И ПАРАМЕТРЫ
 ********************************************************/

// Пины двигателей
#define ENA_RIGHT 5
#define ENA_LEFT  4

// Таймаут смерти робота (20 секунд)
#define ROBOT_LIFETIME_MS 24000

/********************************************************
 *  ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
 ********************************************************/

// Объекты двигателей
GStepper<STEPPER2WIRE> rightStepper(800, 23, 22);
GStepper<STEPPER2WIRE> leftStepper(800, 19, 18);

// Состояния движения
bool movementStarted = false;
bool waitingAfterStart = false;
uint32_t waitStartTime = 0;
uint32_t robotStartTime = 0;       // Время, когда кнопка была отпущена
bool robotIsDead = false;          // Флаг смерти робота
int movePhase = 0;                 // 0 = вперёд, 1 = назад

// Состояние от лидара
bool lidarDanger = false;
bool dangerLogged = false;         // Чтобы логировать "DANGER" только один раз

// Параметры движения
bool phase0Complete = false;
bool phase1Complete = false;

/********************************************************
 *  ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
 ********************************************************/

bool readBytesWithTimeout(HardwareSerial &ser, uint8_t *buffer, size_t length, uint32_t timeout_ms = 500) {
  uint32_t start = millis();
  size_t count = 0;

  while (count < length) {
    if (ser.available()) {
      buffer[count++] = ser.read();
    }
    if (millis() - start > timeout_ms) {
      return false;  // Истёк таймаут, данные не успели прийти
    }
  }
  return true;
}

// Структура пакета лидара: 4 байта заголовка, затем 32 байта данных
static const uint8_t LIDAR_HEADER[] = { 0x55, 0xAA, 0x03, 0x08 };
static const uint8_t LIDAR_HEADER_LEN = 4;
static const uint8_t LIDAR_BODY_LEN = 32;

// Пороговые расстояния (в миллиметрах) и время залипания аварии
#define ALARM_DIST 200     // Менее 400 мм -> сектор в красном цвете
#define WARNING_DIST 350   // Менее 650 мм (но >= 400 мм) -> жёлтый
#define ALARM_HOLD_MS 300  // Время (мс), которое сектор будет «залипать» в красным
#define SECTOR_OFFSET 1    // Cдвиг секторов (0..11)

// Храним текущее измеренное расстояние по каждому из 12 секторов.
static float sectorDistances[12] = { 0.0f };

// Время последнего обновления данных сектора (в миллисекундах).
static uint32_t sectorUpdateTime[12] = { 0 };

// Время, до которого сектор должен находиться в состоянии «тревоги» (красный цвет).
static uint32_t sectorAlarmUntil[12] = { 0 };

// Константа, обозначающая «нет данных».
static const float NO_VALUE = 99999.0f;

bool waitForHeader(HardwareSerial &ser) {
  uint8_t matchPos = 0;
  uint32_t start = millis();

  // Пытаемся «выровнять» поток байт на заголовок (4 байта)
  while (true) {
    if (ser.available()) {
      uint8_t b = ser.read();
      if (b == LIDAR_HEADER[matchPos]) {
        // Совпало очередное ожидаемое значение заголовка
        matchPos++;
        if (matchPos == LIDAR_HEADER_LEN) {
          // Все байты заголовка совпали
          return true;
        }
      } else {
        // Сброс, если последовательность прервалась
        matchPos = 0;
      }
    }
    // Если долго не приходит корректный заголовок, выходим с false
    if (millis() - start > 100) {
      return false;
    }
  }
}

float decodeAngle(uint16_t rawAngle) {
  float angleDeg = (float)(rawAngle - 0xA000) / 64.0f;
  while (angleDeg < 0) angleDeg += 360.0f;
  while (angleDeg >= 360) angleDeg -= 360.0f;
  return angleDeg;
}

int angleToSector(float angleDeg) {
  // Сдвигаем угол на +15°, чтобы 0-й сектор приходился примерно на зону 345..15
  float shifted = angleDeg + 15.0f;
  while (shifted < 0) shifted += 360.0f;
  while (shifted >= 360) shifted -= 360.0f;

  // Делим на 30°, получаем индекс сектора
  int sector = (int)(shifted / 30.0f) % 12;
  sector = (sector + SECTOR_OFFSET) % 12;
  return sector;
}

bool parseAndProcessPacket() {
  // 1) Дожидаемся заголовка лидара
  if (!waitForHeader(Serial1)) {
    return false;  // Заголовок не найден, пропускаем
  }

  // 2) Считываем тело пакета из 32 байт
  uint8_t buffer[LIDAR_BODY_LEN];
  if (!readBytesWithTimeout(Serial1, buffer, LIDAR_BODY_LEN, 500)) {
    return false;
  }

  // 3) Извлекаем общие данные пакета
  uint16_t rotationSpeedTmp = buffer[0] | (buffer[1] << 8);
  uint16_t startAngleTmp = buffer[2] | (buffer[3] << 8);
  float startAngleDeg = decodeAngle(startAngleTmp);

  // Данные о расстояниях и интенсивностях занимают 8 групп по 3 байта
  uint8_t offset = 4;
  uint16_t distances[8];
  uint8_t intensities[8];
  for (int i = 0; i < 8; i++) {
    distances[i] = (buffer[offset] | (buffer[offset + 1] << 8));
    intensities[i] = buffer[offset + 2];
    offset += 3;
  }

  // В самом конце пакета — «конечный угол» (2 байта)
  uint16_t endAngleTmp = buffer[offset] | (buffer[offset + 1] << 8);
  float endAngleDeg = decodeAngle(endAngleTmp);

  // Если конечный угол «меньше» начального, то добавим 360
  if (endAngleDeg < startAngleDeg) {
    endAngleDeg += 360.0f;
  }

  // 4) Вычисляем углы для всех 8 точек внутри пакета
  float angleRange = endAngleDeg - startAngleDeg;
  float angleInc = angleRange / 8.0f;
  float packetAngles[8];
  for (int i = 0; i < 8; i++) {
    float angle = startAngleDeg + i * angleInc;
    // Нормализуем в [0..360)
    while (angle < 0) angle += 360.0f;
    while (angle >= 360) angle -= 360.0f;
    packetAngles[i] = angle;
  }

  // 5) Создаём в ременный массив, где соберём минимальные расстояния по секторам
  float tempSectorMin[12];
  for (int s = 0; s < 12; s++) {
    tempSectorMin[s] = NO_VALUE;  // Изначально никаких данных
  }

  // 6) Проходим по всем 8 точкам пакета, фильтруя по интенсивности
  for (int i = 0; i < 8; i++) {
    if (intensities[i] > 15) {
      int sectorIndex = angleToSector(packetAngles[i]);
      float dist = (float)distances[i];
      // Запоминаем минимальное расстояние на сектор
      if (dist < tempSectorMin[sectorIndex]) {
        tempSectorMin[sectorIndex] = dist;
      }
    }
  }

  // 7) Обновляем глобальный массив расстояний и время их обновления
  uint32_t now = millis();
  for (int s = 0; s < 12; s++) {
    if (tempSectorMin[s] != NO_VALUE) {
      sectorDistances[s] = tempSectorMin[s];
      sectorUpdateTime[s] = now;
    }
  }

  // 8) Если сектор не обновлялся более 500 мс, считаем, что данных по нему нет
  for (int s = 0; s < 12; s++) {
    if ((now - sectorUpdateTime[s]) > 500) {
      sectorDistances[s] = NO_VALUE;
    }
  }

  // 9) Обработка «залипания» красного: если новое расстояние < ALARM_DIST,
  //    продлеваем время «AlarmUntil».
  for (int s = 0; s < 12; s++) {
    float dist = sectorDistances[s];
    if (dist != NO_VALUE && dist < ALARM_DIST) {
      sectorAlarmUntil[s] = now + ALARM_HOLD_MS;
    }
  }

  // 10) Проверяем, есть ли хотя бы один сектор в состоянии "тревога"
  int anyDanger = 0; // Нет опасности изначально
  for (int s = 0; s < 12; s++) {
    float dist = sectorDistances[s];

    if (dist != NO_VALUE && dist < ALARM_DIST) {
      anyDanger = 1;
      break; // Достаточно одного совпадения
    }

    // Также проверяем "залипание" в тревоге
    if (millis() < sectorAlarmUntil[s]) {
      anyDanger = 1;
      break;
    }
  }

  // Сохраняем состояние в глобальную переменную
  lidarDanger = anyDanger;

  return true;  // Пакет успешно обработан
}

void setup() {
  // Инициализация кнопки
  pinMode(2, INPUT_PULLUP);

  // Инициализация двигателей
  pinMode(ENA_RIGHT, OUTPUT);
  pinMode(ENA_LEFT, OUTPUT);

  // Реверсим левый мотор
  leftStepper.reverse(1);

  rightStepper.setRunMode(FOLLOW_POS);
  rightStepper.setAcceleration(600);
  rightStepper.setMaxSpeed(2000);
  digitalWrite(ENA_RIGHT, LOW); // Включаем моторы

  leftStepper.setRunMode(FOLLOW_POS);
  leftStepper.setAcceleration(600);
  leftStepper.setMaxSpeed(2000);
  digitalWrite(ENA_LEFT, LOW);

  // Инициализация Serial для лидара
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("Robot ready. Release button on pin 2 to start. Waiting 5 seconds before moving.");
}

void loop() {
  // Если робот "умер", он больше не делает ничего
  if (robotIsDead) {
    return;
  }

  // Проверяем, была ли отпущена кнопка
  if (!movementStarted && !digitalRead(2)) { // Инвертируем кнопку: если LOW (нажата), то не запускаем
    movementStarted = true;
    robotStartTime = millis(); // Фиксируем время начала
    waitingAfterStart = true;
    waitStartTime = millis();
    Serial.println("Button released. Waiting 5 seconds before moving...");
  }

  // Проверяем, прошло ли 20 секунд с момента отпускания кнопки
  if (movementStarted && (millis() - robotStartTime >= ROBOT_LIFETIME_MS)) {
    Serial.println("Robot died due to timeout!");
    digitalWrite(ENA_RIGHT, HIGH); // Выключаем моторы
    digitalWrite(ENA_LEFT, HIGH);
    robotIsDead = true;
    return;
  }

  // Если идёт ожидание после отпускания кнопки
  if (waitingAfterStart) {
    if (millis() - waitStartTime >= 5000) { // 5 секунд
      waitingAfterStart = false;
      rightStepper.setTarget(7000);  // Вперёд
      leftStepper.setTarget(7000);
      Serial.println("Moving forward...");
    }
    // Продолжаем парсить лидар, но не смотрим на опасность
    parseAndProcessPacket();
    return;
  }

  // Если движение началось
  if (movementStarted && !waitingAfterStart) {
    // Проверяем, не опасно ли двигаться
    if (lidarDanger) {
      // Останавливаем моторы
      rightStepper.setTarget(rightStepper.getCurrent());
      leftStepper.setTarget(leftStepper.getCurrent());
      if (!dangerLogged) {
        Serial.println("DANGER: Motors stopped!");
        dangerLogged = true;
      }
    } else {
      // Сбрасываем флаг, если опасность прошла
      dangerLogged = false;
      // Продолжаем движение
      rightStepper.tick();
      leftStepper.tick();
    }

    // Проверяем, закончилась ли первая фаза (вперёд)
    if (movePhase == 0 && rightStepper.getCurrent() >= 1200) { // Вперёд
      movePhase = 1;
      phase0Complete = true;
      Serial.println("Phase 0 complete. Moving back...");
      rightStepper.setTarget(-300); // Назад
      leftStepper.setTarget(-300);
    }

    // Проверяем, закончилась ли вторая фаза (назад)
    if (movePhase == 1 && rightStepper.getCurrent() <= -300) { // Назад
      phase1Complete = true;
      Serial.println("Phase 1 complete. Movement finished.");
      movementStarted = false;
      movePhase = 0;
    }
  }

  // Проверяем данные лидара
  parseAndProcessPacket();
}
