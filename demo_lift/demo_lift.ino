#include <GyverStepper.h>
#define ENA 26
#define STEPS_PER_REV 800      // Шагов на оборот
#define MM_PER_REV 16.07         // Миллиметров на оборот (настройте под вашу систему)
#define STEPS_PER_MM (STEPS_PER_REV / MM_PER_REV)
#define MIN_POSITION 0.0        // Минимальная позиция в мм
#define MAX_POSITION 200.0      // Максимальная позиция в мм
#define SERIAL_BAUD 115200      // Скорость Serial порта

GStepper<STEPPER2WIRE> stepper(STEPS_PER_REV, 12, 13);
char serialBuffer[20];           // Буфер для приема данных
byte bufferIndex = 0;

void setup() {
  pinMode(ENA, OUTPUT);
  stepper.setRunMode(FOLLOW_POS);
  stepper.setAcceleration(600);
  stepper.setMaxSpeed(2000);
  digitalWrite(ENA, LOW);
  
  // Инициализация Serial
  Serial.begin(SERIAL_BAUD);
  Serial.println("Stepper Controller Ready");
  Serial.print("Min: "); Serial.print(MIN_POSITION);
  Serial.print(" mm | Max: "); Serial.println(MAX_POSITION);
  Serial.println("Send position in mm (0 = disable motor)");
}

// Функция установки позиции в миллиметрах
void setTargetPosition(float mm) {
  if (mm == 0.0f) {
    digitalWrite(ENA, HIGH);  // Обесточиваем мотор
    Serial.println("Motor disabled");
  } else {
    // Ограничиваем позицию пределами рабочей зоны
    mm = constrain(mm, MIN_POSITION, MAX_POSITION);
    
    digitalWrite(ENA, LOW);  // Включаем мотор если был выключен
    long targetSteps = mm * STEPS_PER_MM;
    stepper.setTarget(targetSteps);
    
    Serial.print("Moving to: ");
    Serial.print(mm, 1);
    Serial.println(" mm");
  }
}

// Обработка входящих данных через Serial
void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Обработка конца строки
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0'; // Завершаем строку
        
        // Проверяем, что строка содержит числовое значение
        char* endptr;
        float pos = strtof(serialBuffer, &endptr);
        
        if (endptr != serialBuffer) { // Успешное преобразование
          setTargetPosition(pos);
        } else {
          Serial.println("Error: Invalid number format");
        }
      }
      bufferIndex = 0; // Сброс буфера
      return;
    }
    
    // Сохраняем символ в буфер (с защитой от переполнения)
    if (bufferIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[bufferIndex++] = c;
    } else {
      Serial.println("Error: Input buffer overflow");
      bufferIndex = 0;
    }
  }
}

void loop() {
  stepper.tick();       // Обязательный вызов для работы мотора
  handleSerialInput();  // Обработка Serial команд
}
