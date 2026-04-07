#include <GyverStepper.h>
#include <ESP32Servo.h>

// === ПИНЫ ===
#define L_WHEEL_ENA 4
#define L_WHEEL_STP 19
#define L_WHEEL_DIR 18
#define R_WHEEL_ENA 5
#define R_WHEEL_STP 23
#define R_WHEEL_DIR 22
#define L_LIFT_ENA 26
#define L_LIFT_STP 12
#define L_LIFT_DIR 13
#define R_LIFT_ENA 25
#define R_LIFT_STP 27
#define R_LIFT_DIR 14

// === КОНСТАНТЫ ===
const float WHEEL_DIAMETER_MM = 91.0;
const float STEPS_PER_REVOLUTION = 800.0;
const float MM_TO_M = 0.001;
const float WHEEL_BASE_M = 0.207;  // Пример: 30 см между колёсами

// Коэффициент для перевода м/с в шаги/сек
const float SPEED_M_S_TO_STEPS_S = STEPS_PER_REVOLUTION / (PI * WHEEL_DIAMETER_MM * MM_TO_M);

// Пины сервоприводов (RR, RL, LR, LL)
const int servoPins[] = { 32, 33, 15, 17 };

// === ОБЪЕКТЫ ===
GStepper<STEPPER2WIRE> l_wheel(800, L_WHEEL_STP, L_WHEEL_DIR);
GStepper<STEPPER2WIRE> r_wheel(800, R_WHEEL_STP, R_WHEEL_DIR);
GStepper<STEPPER2WIRE> l_lift(800, L_LIFT_STP, L_LIFT_DIR);
GStepper<STEPPER2WIRE> r_lift(800, R_LIFT_STP, R_LIFT_DIR);

Servo servos[4];

// === ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ===
bool l_lift_powered = true, r_lift_powered = true;
bool servo_attached[4] = { true, true, true, true };
int servosTargetPos[4] = { 90, 90, 90, 90 };
int servosCurrentPos[4] = { 90, 90, 90, 90 };
uint32_t servosTimer[4] = { 0, 0, 0, 0 };
int sDelay = 10;
int servosDelay[4] = { sDelay, sDelay, sDelay, sDelay };

void setup() {
  Serial.begin(115200);

  // Настройка пинов enable
  pinMode(L_WHEEL_ENA, OUTPUT);
  pinMode(R_WHEEL_ENA, OUTPUT);
  pinMode(L_LIFT_ENA, OUTPUT);
  pinMode(R_LIFT_ENA, OUTPUT);

  // Включаем моторы (LOW = активен)
  digitalWrite(L_WHEEL_ENA, LOW);
  digitalWrite(R_WHEEL_ENA, LOW);
  digitalWrite(L_LIFT_ENA, LOW);
  digitalWrite(R_LIFT_ENA, LOW);

  // Настройка моторов
  l_wheel.setRunMode(KEEP_SPEED);
  l_wheel.setAcceleration(600);
  l_wheel.setMaxSpeed(2000);

  r_wheel.setRunMode(KEEP_SPEED);
  r_wheel.setAcceleration(600);
  r_wheel.setMaxSpeed(2000);

  l_lift.setRunMode(FOLLOW_POS);
  l_lift.setAcceleration(600);
  l_lift.setMaxSpeed(2000);

  r_lift.setRunMode(FOLLOW_POS);
  r_lift.setAcceleration(600);
  r_lift.setMaxSpeed(2000);

  // Настройка серв
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], 500, 2500);
    servos[i].write(servosCurrentPos[i]);
  }
}

void loop() {
  l_wheel.tick();
  r_wheel.tick();
  l_lift.tick();
  r_lift.tick();
  servoPosControl();
  handleUARTCommand();
}

void setWheelSpeed(int side, float speed_m_s) {
  float speed_steps_s = speed_m_s * SPEED_M_S_TO_STEPS_S;
  int reverce = -1 + 2 * side;
  auto& motor = (side == 0) ? l_wheel : r_wheel;
  motor.setSpeed(speed_steps_s);
}

void setLiftPosition(int side, float mm) {
  auto& motor = (side == 2) ? l_lift : r_lift;
  bool& powered = (side == 2) ? l_lift_powered : r_lift_powered;
  int ena_pin = (side == 2) ? L_LIFT_ENA : R_LIFT_ENA;

  if (mm == 0.0f) {
    digitalWrite(ena_pin, HIGH);  // Отключить питание
    powered = false;
    Serial.println(side == 2 ? "Left lift disabled" : "Right lift disabled");
  } else {
    if (!powered) {
      digitalWrite(ena_pin, LOW); // Включить питание
      powered = true;
    }
    float stepsPerMM = 800 / 16.07; // Пример: 16.07 mm/rev
    if (side == 3) stepsPerMM = 800 / 16.05; // Для правого подъёмника
    mm = constrain(mm, 0.0, 200.0);
    long targetSteps = mm * stepsPerMM;
    motor.setTarget(-targetSteps);
    Serial.printf("%s moving to: %.1f mm\n", (side == 2 ? "Left lift" : "Right lift"), mm);
  }
}

void setRobotMotion(float linear, float angular) {
  float v_left  = linear - (angular * WHEEL_BASE_M / 2.0);
  float v_right = linear + (angular * WHEEL_BASE_M / 2.0);

  l_wheel.setSpeed(-v_left * SPEED_M_S_TO_STEPS_S);
  r_wheel.setSpeed(v_right * SPEED_M_S_TO_STEPS_S);
}

void setServoAngle(int servoNum, int angle) {
  if (servoNum >= 0 && servoNum < 4) {
    if (angle == -1) {  // Команда на отключение
      if (servo_attached[servoNum]) {
        servos[servoNum].detach();
        servo_attached[servoNum] = false;
        Serial.printf("Servo %d detached\n", servoNum);
      }
    } else if (angle >= 0 && angle <= 180) {  // Команда на угол
      if (!servo_attached[servoNum]) {
        servos[servoNum].attach(servoPins[servoNum], 500, 2500);
        servo_attached[servoNum] = true;
      }
      servosTargetPos[servoNum] = angle;
    }
  }
}

void servoPosControl() {
  for (int i = 0; i < 4; i++) {
    if (servo_attached[i] && (millis() - servosTimer[i] > servosDelay[i])) {
      if (servosCurrentPos[i] != servosTargetPos[i]) {
        int dir = (servosCurrentPos[i] < servosTargetPos[i]) ? 1 : -1;
        servosCurrentPos[i] += dir;
        servos[i].write(servosCurrentPos[i]);
      }
      servosTimer[i] = millis();
    }
  }
}
void handleUARTCommand() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    int firstComma = cmd.indexOf(',');
    if (firstComma == -1) return;

    int target = cmd.substring(0, firstComma).toInt();
    String rest = cmd.substring(firstComma + 1);

    // Обработка команды 8: 8,линейная,угловая
    if (target == 8) {
      int secondComma = rest.indexOf(',');
      if (secondComma != -1) {
        float linear = rest.substring(0, secondComma).toFloat();
        float angular = rest.substring(secondComma + 1).toFloat();
        setRobotMotion(linear, angular);
        return;
      }
    }

    float value = rest.toFloat();

    switch (target) {
      case 0: setWheelSpeed(0, value); break;   // Левое колесо
      case 1: setWheelSpeed(1, value); break;   // Правое колесо
      case 2: setLiftPosition(2, value); break; // Левый подъёмник
      case 3: setLiftPosition(3, value); break; // Правый подъёмник
      case 4 ... 7: setServoAngle(target - 4, (int)value); break;
      // case 9: больше не нужен
      default: Serial.println("Invalid target");
    }
  }
}
