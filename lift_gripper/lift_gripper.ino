#include <GyverStepper.h>

GStepper<STEPPER2WIRE> stepper(800, 3, 4, 2);

// Переменные для UART
String inputString = "";
bool stringComplete = false;

// Коэффициент преобразования мм в шаги (настройте под вашу механику)
const float MM_TO_STEPS = 16.0;  // 1 мм = 160 шагов (пример)

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
  stepper_right.setMaxSpeed(400);
  stepper_right.setAcceleration(300);

  stepper_left.setRunMode(KEEP_SPEED);
  stepper_left.setRunMode(FOLLOW_POS);
  stepper_left.setMaxSpeed(400);
  stepper_left.setAcceleration(300);

  pinMode(26, OUTPUT);
  digitalWrite(26, 1);
}


void loop() {
  stepper_left.tick();
  stepper_right.tick();
  lift_gripper(200,0);
  lift_gripper(200,1);
  delay(1000);
  lift_gripper(0,0);
  lift_gripper(0,1);
}

// Функция обработки серийных данных
void lift_gripper(int value; bool motor) {
  if (motor == 0) {
    if (value == 0) {
      // Выключаем питание мотора
      digitalWrite(26, 0);
      stepper_left.setTarget(1, RELATIVE);
      Serial.println("Motors power OFF");

    } else if (value >= 1 && value <= 255) {
      // Включаем питание и устанавливаем позицию

      // Преобразуем миллиметры в шаги
      long targetSteps = value * MM_TO_STEPS;

      // Устанавливаем целевую позицию
      stepper_left.setTarget(targetSteps);
      digitalWrite(26, 1);
      Serial.print("Moving Left to: ");
      Serial.print(value);
      Serial.print(" mm (");
      Serial.print(targetSteps);
      Serial.println(" steps)");
    } else {
      Serial.println("Invalid value! Use 0-255");
    }
  }
  if (motor == 1) {
    if (value == 0) {
      // Выключаем питание мотора
      digitalWrite(26, 0);
      stepper_right.setTarget(1, RELATIVE);
      Serial.println("Motor power OFF");

    } else if (value >= 1 && value <= 255) {
      // Включаем питание и устанавливаем позицию

      // Преобразуем миллиметры в шаги
      long targetSteps = value * MM_TO_STEPS;

      // Устанавливаем целевую позицию
      stepper_right.setTarget(targetSteps);
      digitalWrite(26, 1);
      Serial.print("Moving Right to: ");
      Serial.print(value);
      Serial.print(" mm (");
      Serial.print(targetSteps);
      Serial.println(" steps)");
    } else {
      Serial.println("Invalid value! Use 0-255");
    }
  }
}

