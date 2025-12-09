#include <GyverStepper.h>
// Переменные для UART
String inputString = "";
bool stringComplete = false;

// Коэффициент преобразования мм в шаги (настройте под вашу механику)
const float MM_TO_STEPS = 32.5;  // 1 мм = 160 шагов (пример)

#include <GyverStepper.h>
GStepper<STEPPER2WIRE> stepper_left(800, 14, 27);
GStepper<STEPPER2WIRE> stepper_right(800, 13, 12);
// GStepper<STEPPER2WIRE> stepper_left(800, 12, 11);
// GStepper<STEPPER2WIRE> stepper_right(800, 9, 10);



void setup() {
  Serial.begin(115200);

  // enable
  pinMode(4, OUTPUT);
  digitalWrite(4, 0);

  stepper_right.setRunMode(KEEP_SPEED);
  stepper_right.setRunMode(FOLLOW_POS);
  stepper_right.setMaxSpeed(2000);
  stepper_right.setAcceleration(2400);

  stepper_left.setRunMode(KEEP_SPEED);
  stepper_left.setRunMode(FOLLOW_POS);
  stepper_left.setMaxSpeed(2000);
  stepper_left.setAcceleration(2400);

  pinMode(26, OUTPUT);
  digitalWrite(26, 1);
}
void setMotorPosition(int motorSelector, float position_mm) {
  // Логирование входных параметров
  Serial.print("setMotorPosition: motor=");
  Serial.print(motorSelector);
  Serial.print(", pos=");
  Serial.print(position_mm, 1);  // 1 знак после запятой
  Serial.println("mm");


  int32_t steps = -static_cast<int32_t>(position_mm * MM_TO_STEPS);

  switch (motorSelector) {
    case 1:  // Только левый мотор
      if (position_mm == 0.0f) {
        Serial.println("  -> Disabling LEFT motor");
        digitalWrite(26, 1);  // Обесточиваем (пин 26)
      } else {
        Serial.print("  -> Moving LEFT motor to ");
        Serial.print(steps);
        Serial.println(" steps");
        stepper_left.setTarget(steps);
        digitalWrite(26, 0);  // Включаем (пин 26)
      }
      break;
    case 2:  // Только правый мотор
      if (position_mm == 0.0f) {
        Serial.println("  -> Disabling RIGHT motor");
        digitalWrite(26, 1);  // Обесточиваем (пин 26)
      } else {
        Serial.print("  -> Moving RIGHT motor to ");
        Serial.print(steps);
        Serial.println(" steps");
        stepper_right.setTarget(steps);
        digitalWrite(26, 0);  // Включаем (пин 26)
      }
      break;
    case 3:  // Оба мотора
      if (position_mm == 0.0f) {
        Serial.println("  -> Disabling BOTH motors");
        digitalWrite(26, 1);  // Обесточиваем оба (пин 26)
      } else {
        Serial.print("  -> Moving BOTH motors to ");
        Serial.print(steps);
        Serial.println(" steps");
        stepper_left.setTarget(steps);
        stepper_right.setTarget(steps);
        digitalWrite(26, 0);  // Включаем (пин 26)
      }
      break;
    default:
      Serial.println("  -> ERROR: Invalid motor selector");
      break;
  }
}
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;

    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void processSerialCommand() {
  if (stringComplete) {
    // Удаляем символы новой строки и пробелы в начале/конце
    inputString.trim();

    // Парсим строку. Ожидаем формат: "MOTOR,POSITION"
    // Например: "3,100.5" - оба мотора на 100.5 мм
    int commaIndex = inputString.indexOf(',');

    if (commaIndex > 0) {
      String motorStr = inputString.substring(0, commaIndex);
      String posStr = inputString.substring(commaIndex + 1);

      int motorSelector = motorStr.toInt();
      float position = posStr.toFloat();

      Serial.print("Received command: motor=");
      Serial.print(motorSelector);
      Serial.print(", position=");
      Serial.print(position, 1);
      Serial.println("mm");

      // Вызываем нашу функцию управления моторами
      setMotorPosition(motorSelector, position);
    } else {
      Serial.println("Invalid command format. Expected: MOTOR,POSITION");
    }

    // Сбрасываем строку и флаг
    inputString = "";
    stringComplete = false;
  }
}
void loop() {
  processSerialCommand(); // Обработка Serial команд
  bool stepper_left_is_moving = stepper_left.tick();
  bool stepper_right_is_moving = stepper_right.tick();
  if (!stepper_right_is_moving and !stepper_left_is_moving) {
  }
}